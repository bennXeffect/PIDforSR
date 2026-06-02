# PIDforSR

PID temperature controller for the heating chamber of a **FLsun Speed Racer** 3D printer.

## What it does

Controls a 100V / 300W heating element inside the printer's enclosure chamber using a PID feedback loop. A DS18B20 temperature sensor reads the chamber temperature, and a relay switches the heater on/off within a time-proportioned window to maintain the target setpoint.

The setpoint is adjusted via a potentiometer (25–80 °C range). A single button toggles the heater on or off. A small SSD1306 OLED display shows the target and actual temperatures along with the current state. Three WS2812B LEDs provide a visual status indicator (breathing blue when idle, purple when active, red center LED when the heater is firing).

## Hardware

- **Microcontroller**: ESP8266-based board (e.g. Wemos D1 mini)
- **Heater**: 100V 300W chamber heater, switched by a relay on pin D4
- **Temperature sensor**: DS18B20 on pin D5 (OneWire)
- **Display**: SSD1306 128×32 OLED via I2C
- **LEDs**: 3× WS2812B on pin D2
- **Input**: Potentiometer on A0 (setpoint), push-button on D6

## PID parameters

| Parameter | Value |
|-----------|-------|
| Kp | 2 |
| Ki | 5 |
| Kd | 1 |
| Window size | 3500 ms |

## Build

Uses [PlatformIO](https://platformio.org/). Open the project and select the appropriate environment for your board, then build and upload.
