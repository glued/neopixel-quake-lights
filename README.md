# Neopixel Earthquake Lights

Ambient RGB LED lights change via [USGS Earthquake GeoJSON data](http://earthquake.usgs.gov/earthquakes/feed/v1.0/summary/1.0_hour.geojson). Colors are based on the magnitude and animation pattern speed changes based on distance from quake.

### What I used

- [Adafruit QT Py ESP32 Pico](https://www.adafruit.com/product/5395)
- [Adafruit NeoPixel Driver BFF Add-On](https://www.adafruit.com/product/5645)
- [Adafruit NeoPixel LED Strip](https://www.adafruit.com/product/3919)

### Setup

- See setup guide for QT Py [here](https://learn.adafruit.com/adafruit-qt-py-esp32-pico/arduino-ide-setup)
- Open `quake-lights.ino` in the Arduino IDE
- Install the following libraries [ArduinoJson](https://arduinojson.org/?utm_source=meta&utm_medium=library.properties) & [Adafruit NeoPixel](https://github.com/adafruit/Adafruit_NeoPixel)
- Change the wifi `ssid` and `pass`
- Update the `home_lat`, `home_lng` to your location
