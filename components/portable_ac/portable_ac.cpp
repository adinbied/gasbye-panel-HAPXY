#include "portable_ac.h"
#include "esphome/core/log.h"
#include <cmath>

namespace esphome {
namespace portable_ac {

static const char *const TAG = "portable_ac";

static const uint32_t FRAME_GAP_MS = 40;      // frames are ~150 ms apart, bytes inside a frame are back-to-back
static const uint32_t RUNON_MS = 20000;       // fan run-on after power-off (panel shows a 20 s countdown)
static const uint8_t ACK_TIMEOUT_FRAMES = 3;  // main frames to wait for the 0x12 echo
static const uint8_t MAX_RETRIES = 5;
static const uint8_t MIN_SET_C = 16;
static const uint8_t MAX_SET_C = 30;
static const uint8_t TURBO_SET_C = 16;

static Fields fields_from_panel(const uint8_t *f) {
  Fields x;
  x.modefan = f[P_MODEFAN];
  x.power = f[P_POWER];
  x.set_c = f[P_SET_C];
  x.set_f = f[P_SET_F];
  x.runon = f[P_RUNON];
  x.turbo = f[P_TURBO];
  return x;
}

uint8_t PortableAC::c_to_f_(uint8_t c) {
  // Matches every value captured from the panel: 16->60, 17->63, 18->64, 19->66, 20->68, 21->70, 22->72
  if (c <= 16)
    return 60;
  return static_cast<uint8_t>(lroundf(c * 1.8f + 32.0f));
}

void PortableAC::setup() {
  for (auto &v : this->raw_last_)
    v = -1;
  this->high_freq_.start();  // keep loop() spinning so forwarding adds ~1 byte of latency

  if (this->current_temp_sensor_ != nullptr) {
    this->current_temp_sensor_->add_on_state_callback([this](float state) {
      this->current_temperature = state;
      this->publish_state();
    });
    if (this->current_temp_sensor_->has_state())
      this->current_temperature = this->current_temp_sensor_->state;
  }
  this->mode = climate::CLIMATE_MODE_OFF;
  this->target_temperature = 18;
  this->fan_mode = climate::CLIMATE_FAN_HIGH;
  this->preset = climate::CLIMATE_PRESET_NONE;
  if (this->override_sensor_ != nullptr)
    this->override_sensor_->publish_state(false);
}

void PortableAC::dump_config() {
  ESP_LOGCONFIG(TAG, "Portable AC interceptor:");
  ESP_LOGCONFIG(TAG, "  Hold override: %s", YESNO(this->hold_override_));
  ESP_LOGCONFIG(TAG, "  Pass-through only: %s", YESNO(this->passthrough_));
  LOG_CLIMATE("  ", "Portable AC", this);
}

void PortableAC::set_passthrough(bool v) {
  this->passthrough_ = v;
  if (v) {
    this->set_override_(false);
    this->send_flag_ = false;
    this->awaiting_ack_ = false;
  }
  ESP_LOGI(TAG, "Pass-through only: %s", v ? "ON (commands disabled)" : "OFF (commands enabled)");
}

void PortableAC::set_override_(bool active) {
  if (this->override_active_ == active)
    return;
  this->override_active_ = active;
  ESP_LOGI(TAG, "Override %s", active ? "active" : "released");
  if (this->override_sensor_ != nullptr)
    this->override_sensor_->publish_state(active);
}

void PortableAC::loop() {
  const uint32_t now = millis();
  uint8_t b;

  // Main board -> panel: forward untouched, parse for status.
  while (this->main_rx_->available()) {
    if (!this->main_rx_->read_byte(&b))
      break;
    this->panel_tx_->write_byte(b);
    this->main_bytes_++;
    this->feed_main_byte_(b, now);
  }

  // Panel -> main board: forward, rewriting fields when overriding.
  while (this->panel_rx_->available()) {
    if (!this->panel_rx_->read_byte(&b))
      break;
    this->panel_bytes_++;
    this->main_tx_->write_byte(this->process_panel_byte_(b, now));
  }

  // Wiring diagnostics: raw byte counts every 10 s, even if no valid frame ever arrives.
  if (now - this->last_stats_ms_ > 10000) {
    this->last_stats_ms_ = now;
    ESP_LOGD(TAG, "Bytes received in total: main board %u, panel %u", this->main_bytes_, this->panel_bytes_);
    ESP_LOGD(TAG, "Frames OK/bad: main board %u/%u, panel %u/%u, panel bytes outside frames %u", this->main_ok_,
             this->main_bad_, this->panel_ok_, this->panel_bad_, this->panel_stray_);
    if (this->have_bad_panel_) {
      ESP_LOGD(TAG, "Last bad panel frame: %s", format_hex_pretty(this->last_bad_panel_, P_FRAME_SIZE).c_str());
      this->have_bad_panel_ = false;
    }
  }
}

// ----------------------------------------------------------------------------
// Panel -> main board
// ----------------------------------------------------------------------------
uint8_t PortableAC::process_panel_byte_(uint8_t b, uint32_t now) {
  // A long gap means any half-received frame is dead. Its rewritten bytes were already sent, but the
  // main board will drop it because the checksum byte never arrived.
  if (this->ppos_ != 0 && (now - this->p_last_byte_ms_) > FRAME_GAP_MS)
    this->ppos_ = 0;
  this->p_last_byte_ms_ = now;

  const uint8_t pos = this->ppos_;

  if (pos == 0) {
    if (b != SYNC0) {
      this->panel_stray_++;
      return b;  // not inside a frame: pass through as-is
    }
    this->p_in_xor_ = 0;
    this->p_out_xor_ = 0;
    this->p_rewrite_ = false;
    this->p_flag_in_frame_ = false;
  } else if (pos == 1 && b != SYNC1) {
    this->ppos_ = 0;
    this->panel_stray_ += 2;
    return b;
  } else if (pos == 2 && b != PANEL_LEN) {
    this->ppos_ = 0;
    this->panel_stray_ += 3;
    return b;
  } else if (pos == P_FLAG) {
    // Decide for the whole frame here. If the panel itself reports a change (button press),
    // never rewrite it: the user wins and we hand control back to the panel.
    const bool user_press = (b == FLAG_CHANGED);
    this->p_rewrite_ = !this->passthrough_ && !user_press && (this->override_active_ || this->send_flag_);
    this->p_flag_in_frame_ = this->p_rewrite_ && this->send_flag_;
  }

  if (pos == P_CHK) {
    const bool valid = ((this->p_in_xor_ ^ CHK_XOR) == b);
    uint8_t out = b;
    if (this->p_rewrite_) {
      out = this->p_out_xor_ ^ CHK_XOR;
      if (!valid)
        out ^= 0xFF;  // input was corrupt: make sure the main board rejects our copy too
    }
    this->pbuf_[pos] = b;
    this->ppos_ = 0;
    if (valid) {
      this->panel_ok_++;
      this->on_panel_frame_(this->pbuf_, this->p_rewrite_);
    } else {
      this->panel_bad_++;
      memcpy(this->last_bad_panel_, this->pbuf_, P_FRAME_SIZE);
      this->have_bad_panel_ = true;
    }
    return out;
  }

  const uint8_t out = (this->p_rewrite_ && pos >= P_FLAG) ? this->rewrite_panel_byte_(pos, b) : b;
  this->pbuf_[pos] = b;
  this->p_in_xor_ ^= b;
  this->p_out_xor_ ^= out;
  this->ppos_ = pos + 1;
  return out;
}

uint8_t PortableAC::rewrite_panel_byte_(uint8_t pos, uint8_t in) {
  const Fields &d = this->desired_;
  switch (pos) {
    case P_FLAG:
      return this->p_flag_in_frame_ ? FLAG_CHANGED : FLAG_IDLE;
    case P_MODEFAN:
      return d.modefan;
    case P_POWER:
      return d.power;
    case P_SET_C:
      return d.set_c;
    case P_SET_F:
      return d.set_f;
    case P_RUNON:
      if (d.power == 0 && this->runon_until_ms_ != 0 &&
          static_cast<int32_t>(millis() - this->runon_until_ms_) < 0)
        return 0x20;
      return 0x00;
    case P_TURBO:
      return d.turbo;
    default:
      return in;
  }
}

void PortableAC::on_panel_frame_(const uint8_t *f, bool rewritten) {
  const Fields pf = fields_from_panel(f);
  const bool changed = !this->have_panel_ || memcmp(&pf, &this->panel_fields_, sizeof(Fields)) != 0;
  this->panel_fields_ = pf;
  this->have_panel_ = true;
  if (changed)
    ESP_LOGD(TAG, "Panel  -> %s", format_hex_pretty(f, P_FRAME_SIZE).c_str());

  if (rewritten) {
    if (this->p_flag_in_frame_) {
      this->send_flag_ = false;
      this->awaiting_ack_ = true;
      this->ack_wait_frames_ = 0;
    }
    return;
  }

  if (f[P_FLAG] == FLAG_CHANGED) {
    // Physical button press on the panel: it becomes the authority again.
    if (this->override_active_ || this->send_flag_)
      ESP_LOGI(TAG, "Panel button pressed, handing control back to the panel");
    this->set_override_(false);
    this->send_flag_ = false;
    this->awaiting_ack_ = false;
    this->desired_ = pf;
    if (pf.turbo == 0)
      this->turbo_saved_set_c_ = pf.set_c;
  }
}

// ----------------------------------------------------------------------------
// Main board -> panel
// ----------------------------------------------------------------------------
void PortableAC::feed_main_byte_(uint8_t b, uint32_t now) {
  if (this->mpos_ != 0 && (now - this->m_last_byte_ms_) > FRAME_GAP_MS)
    this->mpos_ = 0;
  this->m_last_byte_ms_ = now;

  const uint8_t pos = this->mpos_;
  if (pos == 0) {
    if (b != SYNC0)
      return;
    this->m_xor_ = 0;
  } else if ((pos == 1 && b != SYNC1) || (pos == 2 && b != MAIN_LEN)) {
    this->mpos_ = (b == SYNC0) ? 1 : 0;
    this->m_xor_ = SYNC0;
    this->mbuf_[0] = SYNC0;
    return;
  }

  this->mbuf_[pos] = b;
  if (pos == M_FRAME_SIZE - 1) {
    this->mpos_ = 0;
    if ((this->m_xor_ ^ CHK_XOR) == b) {
      this->main_ok_++;
      this->on_main_frame_(this->mbuf_);
    } else {
      this->main_bad_++;
    }
    return;
  }
  this->m_xor_ ^= b;
  this->mpos_ = pos + 1;
}

void PortableAC::on_main_frame_(const uint8_t *f) {
  // Log only structural changes (skip the slowly drifting sensor bytes) to keep logs readable.
  bool changed = !this->have_main_;
  for (uint8_t i = 0; i < M_FRAME_SIZE - 1 && !changed; i++) {
    if (i == 11 || i == 12 || i == 18 || i == 21 || i == 22 || i == 23 || i == 24)
      continue;
    changed = f[i] != this->last_main_[i];
  }
  if (changed)
    ESP_LOGD(TAG, "Main   -> %s", format_hex_pretty(f, M_FRAME_SIZE).c_str());
  memcpy(this->last_main_, f, M_FRAME_SIZE);
  this->have_main_ = true;

  // Command acknowledgement
  if (this->awaiting_ack_) {
    if (f[M_FLAG] == FLAG_CHANGED) {
      this->awaiting_ack_ = false;
      this->retries_ = 0;
      ESP_LOGD(TAG, "Command acknowledged by main board");
      if (!this->hold_override_)
        this->set_override_(false);
    } else if (++this->ack_wait_frames_ > ACK_TIMEOUT_FRAMES) {
      this->awaiting_ack_ = false;
      if (++this->retries_ <= MAX_RETRIES) {
        ESP_LOGW(TAG, "No ack from main board, resending (%u/%u)", this->retries_, MAX_RETRIES);
        this->send_flag_ = true;
      } else {
        ESP_LOGE(TAG, "Main board never acknowledged the command");
        this->retries_ = 0;
      }
    }
  }

  // Sensors
  for (uint8_t i = 0; i < M_FRAME_SIZE; i++) {
    if (this->raw_sensors_[i] != nullptr && this->raw_last_[i] != f[i]) {
      this->raw_last_[i] = f[i];
      this->raw_sensors_[i]->publish_state(f[i]);
    }
  }
  if (this->inverter_sensor_ != nullptr && this->inverter_last_ != f[M_INVERTER]) {
    this->inverter_last_ = f[M_INVERTER];
    this->inverter_sensor_->publish_state(f[M_INVERTER]);
  }
  const bool compressor = f[M_COMPRESSOR] & 0x02;
  if (this->compressor_sensor_ != nullptr && this->compressor_last_ != compressor) {
    this->compressor_last_ = compressor;
    this->compressor_sensor_->publish_state(compressor);
  }
  const bool anti_freeze = f[M_STATUS] & 0x40;
  if (this->anti_freeze_sensor_ != nullptr && this->anti_freeze_last_ != anti_freeze) {
    this->anti_freeze_last_ = anti_freeze;
    this->anti_freeze_sensor_->publish_state(anti_freeze);
  }

  // Climate state, as reported by the main board (the source of truth).
  const bool power = f[M_POWER] & 0x80;
  const uint8_t mode_n = f[M_MODEFAN] >> 4;
  const uint8_t fan_n = f[M_MODEFAN] & 0x07;
  const bool dry = (mode_n == 2);

  climate::ClimateMode new_mode = climate::CLIMATE_MODE_OFF;
  if (power) {
    if (dry) {
      new_mode = climate::CLIMATE_MODE_DRY;
    } else {
      new_mode = climate::CLIMATE_MODE_COOL;
      if (mode_n != 3 && mode_n != 6)
        ESP_LOGW(TAG, "Unknown mode nibble %u (byte 0x%02X), reporting as cool", mode_n, f[M_MODEFAN]);
    }
  }

  climate::ClimateFanMode new_fan;
  if (dry) {
    new_fan = climate::CLIMATE_FAN_AUTO;
  } else if (mode_n == 6 || fan_n == 4) {
    new_fan = climate::CLIMATE_FAN_QUIET;
  } else {
    switch (fan_n) {
      case 0: new_fan = climate::CLIMATE_FAN_AUTO; break;
      case 1: new_fan = climate::CLIMATE_FAN_LOW; break;
      case 2: new_fan = climate::CLIMATE_FAN_MEDIUM; break;
      default: new_fan = climate::CLIMATE_FAN_HIGH; break;
    }
  }

  const climate::ClimatePreset new_preset =
      (f[M_STATUS] & 0x01) ? climate::CLIMATE_PRESET_BOOST : climate::CLIMATE_PRESET_NONE;

  float new_target = this->target_temperature;
  if (!dry && f[M_SET_C] >= MIN_SET_C && f[M_SET_C] <= MAX_SET_C)
    new_target = f[M_SET_C];

  climate::ClimateAction new_action = climate::CLIMATE_ACTION_OFF;
  if (power) {
    if (compressor)
      new_action = dry ? climate::CLIMATE_ACTION_DRYING : climate::CLIMATE_ACTION_COOLING;
    else
      new_action = climate::CLIMATE_ACTION_FAN;
  } else if (compressor) {
    new_action = climate::CLIMATE_ACTION_IDLE;  // still winding down after power-off
  }

  bool dirty = false;
  if (this->mode != new_mode) {
    this->mode = new_mode;
    dirty = true;
  }
  if (!this->fan_mode.has_value() || *this->fan_mode != new_fan) {
    this->fan_mode = new_fan;
    dirty = true;
  }
  if (!this->preset.has_value() || *this->preset != new_preset) {
    this->preset = new_preset;
    dirty = true;
  }
  if (this->target_temperature != new_target) {
    this->target_temperature = new_target;
    dirty = true;
  }
  if (this->action != new_action) {
    this->action = new_action;
    dirty = true;
  }
  if (dirty)
    this->publish_state();
}

// ----------------------------------------------------------------------------
// Home Assistant
// ----------------------------------------------------------------------------
climate::ClimateTraits PortableAC::traits() {
  climate::ClimateTraits t;
  if (this->current_temp_sensor_ != nullptr)
    t.add_feature_flags(climate::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE);
  t.add_feature_flags(climate::CLIMATE_SUPPORTS_ACTION);
  t.add_supported_mode(climate::CLIMATE_MODE_OFF);
  t.add_supported_mode(climate::CLIMATE_MODE_COOL);
  t.add_supported_mode(climate::CLIMATE_MODE_DRY);
  t.add_supported_fan_mode(climate::CLIMATE_FAN_AUTO);
  t.add_supported_fan_mode(climate::CLIMATE_FAN_LOW);
  t.add_supported_fan_mode(climate::CLIMATE_FAN_MEDIUM);
  t.add_supported_fan_mode(climate::CLIMATE_FAN_HIGH);
  t.add_supported_fan_mode(climate::CLIMATE_FAN_QUIET);
  t.add_supported_preset(climate::CLIMATE_PRESET_NONE);
  t.add_supported_preset(climate::CLIMATE_PRESET_BOOST);
  t.set_visual_min_temperature(MIN_SET_C);
  t.set_visual_max_temperature(MAX_SET_C);
  t.set_visual_target_temperature_step(1.0f);
  t.set_visual_current_temperature_step(0.1f);
  return t;
}

void PortableAC::control(const climate::ClimateCall &call) {
  if (this->passthrough_) {
    ESP_LOGW(TAG, "Pass-through only mode is on, ignoring command");
    this->publish_state();  // snap Home Assistant back to the real state
    return;
  }
  if (!this->have_panel_) {
    ESP_LOGW(TAG, "No panel frames seen yet, ignoring command");
    this->publish_state();
    return;
  }

  // Start from whatever is currently in charge.
  Fields d = this->override_active_ ? this->desired_ : this->panel_fields_;
  const bool was_on = d.power & 0x80;

  if (call.get_mode().has_value()) {
    switch (*call.get_mode()) {
      case climate::CLIMATE_MODE_OFF:
        if (was_on)
          this->runon_until_ms_ = millis() + RUNON_MS;
        d.power = 0x00;
        break;
      case climate::CLIMATE_MODE_DRY:
        d.power = 0x80;
        d.modefan = MF_DRY;
        d.turbo = 0x00;
        break;
      case climate::CLIMATE_MODE_COOL:
      default:
        d.power = 0x80;
        if ((d.modefan >> 4) == 2)  // leaving dry
          d.modefan = MF_COOL_HIGH;
        break;
    }
    if (d.power)
      this->runon_until_ms_ = 0;
  }

  if (call.get_fan_mode().has_value()) {
    if ((d.modefan >> 4) == 2) {
      ESP_LOGW(TAG, "Fan speed is fixed in dry mode, ignoring");
    } else {
      switch (*call.get_fan_mode()) {
        case climate::CLIMATE_FAN_AUTO: d.modefan = MF_COOL_AUTO; break;
        case climate::CLIMATE_FAN_LOW: d.modefan = MF_COOL_LOW; break;
        case climate::CLIMATE_FAN_MEDIUM: d.modefan = MF_COOL_MED; break;
        case climate::CLIMATE_FAN_QUIET: d.modefan = MF_COOL_QUIET; break;
        case climate::CLIMATE_FAN_HIGH:
        default: d.modefan = MF_COOL_HIGH; break;
      }
    }
  }

  if (call.get_preset().has_value()) {
    const bool want_turbo = *call.get_preset() == climate::CLIMATE_PRESET_BOOST;
    if (want_turbo && !d.turbo) {
      this->turbo_saved_set_c_ = d.set_c;  // the panel forces 16 C in turbo and restores afterwards
      d.turbo = 0x20;
      d.set_c = TURBO_SET_C;
      d.set_f = c_to_f_(TURBO_SET_C);
    } else if (!want_turbo && d.turbo) {
      d.turbo = 0x00;
      d.set_c = this->turbo_saved_set_c_;
      d.set_f = c_to_f_(d.set_c);
    }
  }

  if (call.get_target_temperature().has_value()) {
    const float t = *call.get_target_temperature();
    const uint8_t c = static_cast<uint8_t>(clamp<long>(lroundf(t), MIN_SET_C, MAX_SET_C));
    if (d.turbo) {
      this->turbo_saved_set_c_ = c;  // applied when turbo is switched off
    } else {
      d.set_c = c;
      d.set_f = c_to_f_(c);
    }
  }

  this->desired_ = d;
  this->set_override_(true);
  this->send_flag_ = true;
  this->retries_ = 0;
  ESP_LOGI(TAG, "Command: modefan=0x%02X power=0x%02X set=%uC/%uF turbo=0x%02X", d.modefan, d.power, d.set_c,
           d.set_f, d.turbo);
}

}  // namespace portable_ac
}  // namespace esphome
