# bessy-b
A replacement controller for a pitboss pellet grill

## Hardware
esp32_relay_4 board
  relay1: fan
  relay2: auger
  relay3: igniter
  relay4: unused?
3.5" TFT SPI 480x320, ST7796U driver, SPI  https://esphome.io/components/display/ili9xxx/ (model: ST7796), https://a.co/d/0i3EBVLF
   Module has onboard level-conversion circuit (5V/3.3V MCU compatible) - no external level shifters needed; VCC at 5V is just for full backlight brightness
   I2C for FT6336U CTC (Capacivive Touch Controller)
Rotary encoder, gpio, https://esphome.io/components/sensor/rotary_encoder/, https://a.co/d/0bie5vDb
MAX31865 RTD Temperature sensor, SPI, https://esphome.io/components/sensor/max31865/, https://a.co/d/07EdVxqQ
ZMCT103C AC Current Sensor, analog, https://a.co/d/072UPmVe
MAX31855 Type K thermocouple sensor , SPI, https://esphome.io/components/sensor/max31855/, https://a.co/d/045DVBZG
ADS1115 4 chan, 16 bit ADC, I2C, https://esphome.io/components/sensor/ads1115/

# IO asssignments

| GPIO | Header | Device           | Signal | Color  | Row | Notes |
|------|--------|------------------|--------|--------|-----|-------|
| GND  | A      |                  | ground | Black  | 1   | |
| 3V3  | A      |                  | 3.3VDC | Orange | 3   | |
| 5V   | B      |                  | 5VDC   | Red    | 5   | |
| 18   | B      | SPI bus (shared) | SCLK   | Yellow | 7   | to all 3 SPI devices |
| 19   | B      | SPI bus (shared) | MISO   | Green  | 8   | TFT doesn't use it;   fine |
| 27   | A      | SPI bus (shared) | MOSI   | Blue   | 9   | MAX31855 doesn't use it; fine |
| 4    | B      | ST7796U (TFT)    | CS     | Grey   | 10  | |
| 14   | A      | ST7796U (TFT)    | RS     | Violet | 11  | |
| 13   | A      | ST7796U (TFT)    | RST    | Brown  | 12  | |
| 5    | B      | ST7796U (TFT)    | BL     | White  | 13  | tie to 3V3 if no PWM dimming |
| 16   | B      | unused           | CS     | Orange | 14  | |
| 17   | B      | MAX31855 (TC)    | CS     | Violet | 15  | |
| 21   | B      | ADS1115 (I²C)    | SDA    | Blue   | 16  | 4.7k pull-up to 3V3 |
| 22   | B      | ADS1115 (I²C)    | SCL    | Green  | 17  | 4.7k pull-up to 3V3 |
| 34   | A      | Analog sensor    | AIN    | White  | 18  | ADC1_CH6, input-only |
| 35   | A      | Rotary encoder   | A      | Grey   | 19  | ext 10k pull-up to 3V3 |
| 39 (SVN) | A  | Rotary encoder   | B      | Brown  | 20  | ext 10k pull-up to 3V3 |
| 36 (SVP) | A  | Rotary encoder   | SW     | Yellow | 21  | ext 10k pull-up to 3V3 |
| 23   | A      | FT6336U (CTP)    | INT    | White  | 25  | |
| 12   | A      | FT6336U (CTP)    | RST    | Violet | 26  | strapping pin; ok as output after boot |

Row is the labeled row on the interposer board

Max31865 c1-10
max31855 c11-19
ads1115 c20-29
zmc c31-38


Existing opening 175 x 90 mm

## tuning code from opus/chat

- id: sc_pot_heating
    mode: restart
    then:
      - logger.log: "sc_pot_heating starting"
      - while:
          condition:
            - switch.is_on: go
          then:
            - if:
                condition: { lambda: 'return !id(tune_active);' }
                then:
                  - script.execute: sc_pid_control
            - logger.log:
                format: "Heating cycle: ON time=%.1fs, OFF time=%.1fs"
                args: ["id(augers_heating_on_time).state", "id(augers_heating_off_time).state"]
            - switch.turn_on: auger
            - delay: !lambda "return id(augers_heating_on_time).state * 1000;"
            - if:
                condition: { lambda: 'return !id(tune_active);' }
                then:
                  - script.execute: sc_pid_control
            - switch.turn_off: auger
            - delay: !lambda "return id(augers_heating_off_time).state * 1000;"


globals:
  - { id: tune_active,     type: bool,     initial_value: 'false' }
  - { id: tune_relay_low,  type: bool,     initial_value: 'false' }
  - { id: tune_last_up_ms, type: uint32_t, initial_value: '0' }
  - { id: tune_start_ms,   type: uint32_t, initial_value: '0' }
  - { id: tune_cmax,       type: float,    initial_value: '0.0' }
  - { id: tune_cmin,       type: float,    initial_value: '0.0' }
  - { id: tune_period_sum, type: float,    initial_value: '0.0' }
  - { id: tune_amp_sum,    type: float,    initial_value: '0.0' }
  - { id: tune_good,       type: int,      initial_value: '0' }
  - { id: tune_peaks_seen, type: int,      initial_value: '0' }
  # ---- config ----
  - { id: tune_cycle_s,  type: float, initial_value: '15.0' }   # match sc_pid_control cycle_time
  - { id: tune_warmup,   type: int,   initial_value: '2' }      # discard first N cycles
  - { id: tune_target,   type: int,   initial_value: '4' }      # average this many good cycles
  - { id: tune_timeout_s,type: float, initial_value: '3600.0' } # abort if no limit cycle forms

number:
  - platform: template
    id: tune_u_high
    name: "Autotune duty HIGH"
    optimistic: true
    min_value: 0.05
    max_value: 0.80
    step: 0.01
    initial_value: 0.35
    restore_value: true
  - platform: template
    id: tune_u_low
    name: "Autotune duty LOW"
    optimistic: true
    min_value: 0.05
    max_value: 0.80
    step: 0.01
    initial_value: 0.12
    restore_value: true
  - platform: template
    id: tune_hyst
    name: "Autotune hysteresis (F)"
    optimistic: true
    min_value: 0.0
    max_value: 15.0
    step: 0.5
    initial_value: 3.0
    restore_value: true  

switch:
  - platform: template
    id: pid_autotune
    name: "PID Autotune (relay)"
    optimistic: true
    turn_on_action:
      - lambda: |-
          id(tune_relay_low)  = false;   // start feeding HIGH (assume we're below setpoint)
          id(tune_last_up_ms) = 0;
          id(tune_start_ms)   = millis();
          id(tune_cmax)       = id(smoker_temp_f).state;
          id(tune_cmin)       = id(smoker_temp_f).state;
          id(tune_period_sum) = 0.0f;
          id(tune_amp_sum)    = 0.0f;
          id(tune_good)       = 0;
          id(tune_peaks_seen) = 0;
          id(tune_active)     = true;
          ESP_LOGI("tune", "Autotune started.");
    turn_off_action:
      - lambda: |-
          id(tune_active) = false;
          ESP_LOGI("tune", "Autotune stopped; PID resumes.");

interval:
  - interval: 2s
    then:
      - lambda: |-
          if (!id(tune_active)) return;
          if (!id(go).state) { id(pid_autotune).turn_off(); return; }

          float t  = id(smoker_temp_f).state;
          float sp = id(smoker_set_temp).state;
          if (isnan(t)) { id(auger).turn_off(); return; }

          uint32_t now = millis();
          if (now - id(tune_start_ms) > (uint32_t)(id(tune_timeout_s) * 1000.0f)) {
            ESP_LOGW("tune", "Timed out with no clean limit cycle; aborting.");
            id(pid_autotune).turn_off();
            return;
          }

          float h  = id(tune_hyst).state;     // relay hysteresis = epsilon in Ku formula
          float uh = id(tune_u_high).state;
          float ul = id(tune_u_low).state;

          // track extrema over the current period
          if (t > id(tune_cmax)) id(tune_cmax) = t;
          if (t < id(tune_cmin)) id(tune_cmin) = t;

          // relay with hysteresis, centered on setpoint
          if (id(tune_relay_low)) {
            if (t < sp - h) {                 // low -> high : one full period completed
              id(tune_relay_low) = false;
              if (id(tune_last_up_ms) != 0) {
                float period = (now - id(tune_last_up_ms)) / 1000.0f;
                float amp    = (id(tune_cmax) - id(tune_cmin)) / 2.0f;
                id(tune_peaks_seen) += 1;
                if (id(tune_peaks_seen) > id(tune_warmup)) {
                  id(tune_period_sum) += period;
                  id(tune_amp_sum)    += amp;
                  id(tune_good)       += 1;
                  ESP_LOGI("tune", "cycle %d: period=%.1fs amp=%.2fF",
                           (int)id(tune_good), period, amp);
                } else {
                  ESP_LOGI("tune", "warmup %d discarded: period=%.1fs amp=%.2fF",
                           (int)id(tune_peaks_seen), period, amp);
                }
              }
              id(tune_last_up_ms) = now;
              id(tune_cmax) = t;
              id(tune_cmin) = t;
            }
          } else {
            if (t > sp + h) id(tune_relay_low) = true;   // high -> low
          }

          // apply relay output through the existing auger-drive contract
          float u   = id(tune_relay_low) ? ul : uh;
          float cyc = id(tune_cycle_s);
          id(augers_heating_on_time).publish_state(u * cyc);
          id(augers_heating_off_time).publish_state((1.0f - u) * cyc);

          // enough good cycles? compute and finish
          if (id(tune_good) >= id(tune_target)) {
            float Tu = id(tune_period_sum) / id(tune_good);
            float a  = id(tune_amp_sum)    / id(tune_good);
            float d  = (uh - ul) / 2.0f;
            float den = a*a - h*h;
            den = (den > 0.0001f) ? sqrtf(den) : a;    // guard: hysteresis >= amplitude
            float Ku = (4.0f * d) / (3.14159265f * den);

            float pb = 3.2f / Ku;      // kp = 1/pb = Ku/3.2
            float ti = 2.2f * Tu;
            float td = Tu / 6.3f;

            ESP_LOGI("tune", "==== AUTOTUNE COMPLETE ====");
            ESP_LOGI("tune", "Ku=%.5f duty/F  Tu=%.1fs  a=%.2fF  d=%.3f  h=%.2fF",
                     Ku, Tu, a, d, h);
            ESP_LOGI("tune", "Tyreus-Luyben ->  pb=%.1f   ti=%.1f   td=%.2f", pb, ti, td);
            ESP_LOGI("tune", "Type those into pid_pb / pid_ti / pid_td.");
            id(pid_autotune).turn_off();   // loop guard hands control back to the PID
          }

Running it:

Light and stabilize at your target (say 225°F) under the existing PID. Watch augers_heating_on_time in the logs once it's holding — on_time / 15 is your steady hold duty.
Set duty HIGH ~0.10–0.15 above that hold duty and duty LOW ~0.10–0.15 below it, both inside [0.05, 0.80], both keeping the fire alive. That centers the swing on the real operating point, which is what makes the extracted Ku valid there. d = (HIGH − LOW)/2 is the relay amplitude the math uses.
Flip PID Autotune on. It bypasses the PID and relays the auger around setpoint; temp oscillates. After 2 discarded + 4 averaged cycles it logs pb / ti / td, turns itself off, and the PID resumes.
Type the three numbers into pid_pb / pid_ti / pid_td.

A few things that'll bite if ignored: if the temp sits entirely above or below setpoint and never crosses, your HIGH/LOW aren't centered — nudge them (live, via the numbers) and restart. Keep the lid shut with a representative meat load and no wild weather so the tune reflects a real cook. And note the applied duty only updates when the auger loop next reads the globals (≤15 s), while the detector samples at 2 s — negligible phase lag on a multi-minute period, but drop tune_cycle_s if you want it tighter.
