# Portable AC interceptor (ESPHome, ESP8266)

An ESP8266 sits between the touch panel and the main board. It forwards both serial
lines and rewrites the panel's command frames when Home Assistant sends a command.
Home Assistant sees it as a normal `climate` entity: Off/Cool/Dry, Auto/Low/Med/High/Quiet
fan, 16–30 °C, and the Boost preset (= turbo).

## Wiring

Only 3 wires run between the panel and the main board: two data lines and ground.
Cut the two data lines (leave ground intact) and splice the ESP in. Both lines are 5 V logic.

| Cut wire end | Through | NodeMCU pin |
|---|---|---|
| Ch1 wire, **main board end** (main TX, 28-byte frames) | 4.7 kΩ series, 10 kΩ from D6 to GND | D6 (GPIO12), software RX |
| Ch0 wire, **main board end** (main RX) | 1 kΩ series | TX (GPIO1), hardware UART0 |
| Ch0 wire, **panel end** (panel TX, 16-byte frames) | 4.7 kΩ series, 10 kΩ from D5 to GND | D5 (GPIO14), software RX |
| Ch1 wire, **panel end** (panel RX) | 1 kΩ series | D4 (GPIO2), hardware UART1 |
| Ground wire (spliced, not cut) | | GND |
| +5 V wire (spliced, not cut) | 470 µF + 100 nF to GND | Vin |

- **Don't use the NodeMCU's RX pin.** It's wired to the USB chip through 470 Ω, and the chip holds
  it high harder than a 4.7 kΩ divider can pull it low, so nothing gets through. Both inputs use
  plain GPIOs instead, and both outputs use hardware transmitters.
- The logs print `Bytes received in total: main board N, panel N` every 10 s. If the main board
  count stays at 0, nothing reaches D6. If a divider node reads near 0 V with the AC on, that AC
  line is open-drain and lost its pull-up in the cut: add 10 kΩ from that AC wire end to 5 V.
- The dividers bring 5 V down to ~3.4 V for the ESP inputs.
- The ESP outputs drive 3.3 V straight into the AC. Most 5 V chips accept that. If the panel or
  main board stops responding even in pass-through mode, swap each 1 kΩ for an NPN inverter
  (10 kΩ base resistor, 4.7 kΩ pull-up to the harness 5 V on the collector) and set `inverted: true` on
  that `tx_pin`.
- Power: splice the harness +5 V wire (leave it uncut) to the NodeMCU's **Vin** pin, never a
  3V3 pin, with a 470–1000 µF electrolytic and a 100 nF ceramic across Vin and GND at the board.
  Check what makes that 5 V first: a TO-220 7805 or a switching module has headroom; a tiny
  TO-92 78L05 (~100 mA) does not, so use a USB charger instead. The "ESP supply voltage"
  sensor shows dips (it often reads 0.2–0.4 V low, so confirm with a meter on 3V3);
  `output_power: 15dB` under `wifi:` reduces the current spikes.
- Once it's wired in, never plug USB into a computer: the two 5 V supplies would meet. Flash over USB first, then update over Wi-Fi.
- At boot the ESP8266 prints a few bytes of ROM garbage on TX. The main board sees a short burst
  of junk with no valid checksum, and the panel link drops for ~1–2 s while the ESP boots.
  Watch whether the main board shows a comms error code during that time.
- Optional fail-safe: a DPDT relay whose normally-closed contacts bridge each cut wire straight
  through, energized from D1 (GPIO5), so the ESP is only in the path while it is running.

## Bring-up

1. Copy `secrets.example.yaml` to `secrets.yaml` and fill it in, then flash over USB
   **before** wiring it in: `esphome run portable-ac.yaml`.
2. Wire it in. It boots in **pass-through only** mode: nothing is modified.
3. Check that the panel still works normally and that the climate entity in Home
   Assistant follows the panel (mode, fan, setpoint, turbo, compressor, inverter level).
4. Turn off the **Pass-through only** switch and try commands from Home Assistant.
5. The experiment to run: after a Home Assistant change, does the **panel display** update?
   - If yes, the panel follows the main board's echo. Great.
   - If no, the panel is showing stale settings. Pressing any panel button will send the
     panel's (stale) full state and hand control back to the panel. That's by design: a
     physical press always wins.
6. Optional second experiment: set `hold_override: false`. Commands are then sent once and
   the ESP goes back to pure pass-through. If the AC keeps the Home Assistant setting, the
   main board only acts on "changed" (0x12) frames and the override never needs holding.

Logs (Wi-Fi, `esphome logs portable-ac.yaml`) print every structural frame change on both
lines at DEBUG level, so you can keep reverse-engineering with it in place.

## Protocol summary

`54 44 LEN payload… CHK`, CHK = XOR of all previous bytes ^ 0x10, 9600 8N1.

Panel → main (16 bytes): `[3]` 11/12 changed flag · `[4]` mode/fan (0x38 auto, 39 low, 3A med,
3B high, 6C quiet, 29 dry) · `[5]` 0x80 power · `[6]` setpoint °C · `[9]` setpoint °F ·
`[13]` 0x20 run-on timer · `[14]` 0x20 turbo.

Main → panel (28 bytes): `[3]` ack echo · `[4..6]` echo · `[8]` 0x02 compressor ·
`[14]` setpoint °F · `[17]` +01 turbo +04 quiet +40 anti-freeze? · `[25]` inverter level ·
`[11,12,18,21,22]` temperature sensors (raw) · `[23,24]` load/fan (raw).

Not captured yet: fan-only and heat modes (if the unit has them), setpoints above 22 °C,
and the 4-bar inverter level.
