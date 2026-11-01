#pragma once

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

namespace esphome {
namespace portable_ac {

// ---------------------------------------------------------------------------
// Wire format (both directions, 9600 8N1):
//   54 44 LEN payload... CHK      CHK = XOR(all previous bytes) ^ 0x10
// Panel  -> main board: LEN 0x0C (16-byte frames)
// Main   -> panel:      LEN 0x18 (28-byte frames)
// ---------------------------------------------------------------------------
static const uint8_t SYNC0 = 0x54;
static const uint8_t SYNC1 = 0x44;
static const uint8_t PANEL_LEN = 0x0C;
static const uint8_t MAIN_LEN = 0x18;
static const uint8_t CHK_XOR = 0x10;
static const uint8_t FLAG_IDLE = 0x11;
static const uint8_t FLAG_CHANGED = 0x12;

// Panel frame offsets
static const uint8_t P_FLAG = 3;     // 11 idle, 12 = state changed
static const uint8_t P_MODEFAN = 4;  // hi nibble mode (3 cool, 6 cool+quiet, 2 dry), lo 3 bits fan
static const uint8_t P_POWER = 5;    // 0x80 on
static const uint8_t P_SET_C = 6;    // setpoint degC
static const uint8_t P_SET_F = 9;    // setpoint degF
static const uint8_t P_RUNON = 13;   // 0x20 during the 20 s fan run-on after power-off
static const uint8_t P_TURBO = 14;   // 0x20 turbo
static const uint8_t P_CHK = 15;

// Main board frame offsets
static const uint8_t M_FLAG = 3;     // echoes 12 one frame after a change = ack
static const uint8_t M_MODEFAN = 4;
static const uint8_t M_POWER = 5;
static const uint8_t M_SET_C = 6;
static const uint8_t M_COMPRESSOR = 8;  // 0x02 compressor running
static const uint8_t M_STATUS = 17;     // +01 turbo, +04 quiet, +40 anti-freeze(?)
static const uint8_t M_INVERTER = 25;   // inverter level / compressor frequency
static const uint8_t M_FRAME_SIZE = MAIN_LEN + 4;
static const uint8_t P_FRAME_SIZE = PANEL_LEN + 4;

// Known mode/fan byte values
static const uint8_t MF_COOL_AUTO = 0x38;
static const uint8_t MF_COOL_LOW = 0x39;
static const uint8_t MF_COOL_MED = 0x3A;
static const uint8_t MF_COOL_HIGH = 0x3B;
static const uint8_t MF_COOL_QUIET = 0x6C;
static const uint8_t MF_DRY = 0x29;

struct Fields {
  uint8_t modefan{MF_COOL_HIGH};
  uint8_t power{0x00};
  uint8_t set_c{18};
  uint8_t set_f{0x40};
  uint8_t runon{0x00};
  uint8_t turbo{0x00};
};

class PortableAC : public climate::Climate, public Component {
 public:
  // Each direction can use its own UART bus. If the separate RX/TX bus isn't set, the
  // main bus is used for both directions.
  void set_mainboard_uart(uart::UARTComponent *u) { this->main_rx_ = this->main_tx_ = u; }
  void set_mainboard_rx_uart(uart::UARTComponent *u) { this->main_rx_ = u; }
  void set_panel_uart(uart::UARTComponent *u) { this->panel_rx_ = this->panel_tx_ = u; }
  void set_panel_tx_uart(uart::UARTComponent *u) { this->panel_tx_ = u; }
  void set_hold_override(bool v) { this->hold_override_ = v; }
  void set_passthrough(bool v);
  bool get_passthrough() const { return this->passthrough_; }
  void set_current_temperature_sensor(sensor::Sensor *s) { this->current_temp_sensor_ = s; }
  void set_inverter_level_sensor(sensor::Sensor *s) { this->inverter_sensor_ = s; }
  void set_compressor_sensor(binary_sensor::BinarySensor *s) { this->compressor_sensor_ = s; }
  void set_anti_freeze_sensor(binary_sensor::BinarySensor *s) { this->anti_freeze_sensor_ = s; }
  void set_override_active_sensor(binary_sensor::BinarySensor *s) { this->override_sensor_ = s; }
  void set_raw_sensor(uint8_t offset, sensor::Sensor *s) {
    if (offset < M_FRAME_SIZE) this->raw_sensors_[offset] = s;
  }

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

 protected:
  climate::ClimateTraits traits() override;
  void control(const climate::ClimateCall &call) override;

  // main board -> panel (forwarded untouched, parsed for status)
  void feed_main_byte_(uint8_t b, uint32_t now);
  void on_main_frame_(const uint8_t *f);
  // panel -> main board (rewritten on the fly when overriding)
  uint8_t process_panel_byte_(uint8_t b, uint32_t now);
  uint8_t rewrite_panel_byte_(uint8_t pos, uint8_t in);
  void on_panel_frame_(const uint8_t *f, bool rewritten);

  void set_override_(bool active);
  static uint8_t c_to_f_(uint8_t c);

  uart::UARTComponent *main_rx_{nullptr};
  uart::UARTComponent *main_tx_{nullptr};
  uart::UARTComponent *panel_rx_{nullptr};
  uart::UARTComponent *panel_tx_{nullptr};
  uint32_t main_bytes_{0};
  uint32_t panel_bytes_{0};
  uint32_t main_ok_{0}, main_bad_{0}, panel_ok_{0}, panel_bad_{0}, panel_stray_{0};
  uint8_t last_bad_panel_[P_FRAME_SIZE]{};
  bool have_bad_panel_{false};
  uint32_t last_stats_ms_{0};
  HighFrequencyLoopRequester high_freq_;

  bool hold_override_{true};
  bool passthrough_{true};

  // panel->main stream state
  uint8_t pbuf_[P_FRAME_SIZE]{};
  uint8_t ppos_{0};
  uint8_t p_in_xor_{0};
  uint8_t p_out_xor_{0};
  bool p_rewrite_{false};
  bool p_flag_in_frame_{false};
  uint32_t p_last_byte_ms_{0};

  // main->panel parse state
  uint8_t mbuf_[M_FRAME_SIZE]{};
  uint8_t mpos_{0};
  uint8_t m_xor_{0};
  uint32_t m_last_byte_ms_{0};
  uint8_t last_main_[M_FRAME_SIZE]{};
  bool have_main_{false};

  // state
  bool have_panel_{false};
  Fields panel_fields_{};     // what the panel itself is asking for
  Fields desired_{};          // what we send while overriding
  bool override_active_{false};
  bool send_flag_{false};     // put 0x12 in the next rewritten frame
  bool awaiting_ack_{false};
  uint8_t ack_wait_frames_{0};
  uint8_t retries_{0};
  uint8_t turbo_saved_set_c_{18};
  uint32_t runon_until_ms_{0};

  sensor::Sensor *current_temp_sensor_{nullptr};
  sensor::Sensor *inverter_sensor_{nullptr};
  binary_sensor::BinarySensor *compressor_sensor_{nullptr};
  binary_sensor::BinarySensor *anti_freeze_sensor_{nullptr};
  binary_sensor::BinarySensor *override_sensor_{nullptr};
  sensor::Sensor *raw_sensors_[M_FRAME_SIZE]{};
  int16_t raw_last_[M_FRAME_SIZE];
  int16_t inverter_last_{-1};
  int8_t compressor_last_{-1};
  int8_t anti_freeze_last_{-1};
};

}  // namespace portable_ac
}  // namespace esphome
