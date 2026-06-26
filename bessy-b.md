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
| 19   | B      | SPI bus (shared) | MISO   | Green  | 8   | TFT doesn't use it; fine |
| 27   | A      | SPI bus (shared) | MOSI   | Blue   | 9   | MAX31855 doesn't use it; fine |
| 4    | B      | ST7796U (TFT)    | CS     | Grey   | 10  | |
| 14   | A      | ST7796U (TFT)    | RS     | Violet | 11  | |
| 13   | A      | ST7796U (TFT)    | RST    | Brown  | 12  | |
| 5    | B      | ST7796U (TFT)    | BL     | White  | 13  | tie to 3V3 if no PWM dimming |
| 16   | B      | MAX31865 (RTD)   | CS     | Orange | 14  | |
| 17   | B      | MAX31855 (TC)    | CS     | Violet | 15  | |
| 21   | B      | ADS1115 (I²C)    | SDA    | Blue   | 16  | 4.7k pull-up to 3V3 |
| 22   | B      | ADS1115 (I²C)    | SCL    | Green  | 17  | 4.7k pull-up to 3V3 |
| 34   | A      | Analog sensor    | AIN    | White  | 18  | ADC1_CH6, input-only |
| 35   | A      | Rotary encoder   | A      | Grey   | 19  | ext 10k pull-up to 3V3 |
| 39 (SVN) | A  | Rotary encoder   | B      | Brown  | 20  | ext 10k pull-up to 3V3 |
| 36 (SVP) | A  | Rotary encoder   | SW     | Yellow | 21  | ext 10k pull-up to 3V3 |
| 10   | A      | FT6336U (CTP)    | INT    | White  | 25  | |
| 11   | A      | FT6336U (CTP)    | RST    | Violet | 26  | |

Row is the labeled row on the interposer board

Max31865 c1-10
max31855 c11-19
ads1115 c20-29
zmc c31-38


Existing opening 175 x 90 mm
