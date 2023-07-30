# LoRa_APRS_Meteo
LoRa APRS Meteostation, IGate and Digipeater firmware for TTGO LoRa32 V2.1 
this is a personal evolution of the original OK2DDS project<br><br>
Features: measures temperature, pressure (with BMP280 or BME280) and humidity (with BME280 or integrated module AHT20+BMP280).
Sensors are automatically recognized without setup.
<br>
<img src="https://github.com/iw1cgw/LoRa_APRS_Meteo_dds-cgw/blob/main/img/sensor.jpg">
<br><br>
It is possible to change the main parameters from the menu using a serial terminal eg Putty or Termite without the need to recompile again.
<br><br>
<img src="https://github.com/iw1cgw/LoRa_APRS_Meteo_dds-cgw/blob/main/img/menu.jpg">
<br>
Also from the TTY terminal, the possibility of checking the operating parameters.
<br><br>
<img src="https://github.com/iw1cgw/LoRa_APRS_Meteo_dds-cgw/blob/main/img/display.jpg">
<br><br>
In the 'bin' directory there is the compiled firmware which can be inoculated using the usual tools or my CGWloader. The firmware must be inoculated starting from location 0x0000
<br><br>
<img src="https://github.com/iw1cgw/LoRa_APRS_Meteo_dds-cgw/blob/main/img/CGWloader.jpg">
<br><br>
Weather data from sensors is sent via APRS/APRS-IS LoRa. Works as a standard APRS LoRa IGate and/or digipeater, reports coverage.<br>
<br><br>
<img src="https://github.com/iw1cgw/LoRa_APRS_Meteo_dds-cgw/blob/main/img/aprsmap.jpg">
<br><br>
Source code is in src/main.cpp, before running your station please check:<br>
- you have a valid HAM radio license
- you have edited menu configuration
<br><br>
If you need help or have any questions or suggestions fot this realise, please reach:iw1cgw(at)libero.it

For running the temperature/pressure measurement, you will need to use BMP280+AHT20 or BMP280 or BME280 sensor, sensors are automatically recognized without setup.
Please solder VCC to 3.3V pin, GND to GND, SCL to IO22 and SDA to IO21.<br>

The BMP280/BME280 pressure sensor does not require calibration, but the height in meters of the weather station must be set correctly from the menu to obtain a reliable SLM pressure.<br>

The detected temperature can be compensated +/- 5 Celsius from the menu; if the AHT20+BMP280 module is used, it is possible to choose which of the 2 available thermometers to use.<br>

If the meteostation is connected to Wi-Fi, it runs a tiny webserver. Some of the endpoints are even suitable for browser use.<br>Access your station dashboard with entering your station IP to your browser. For accessing from outside your home network, make sure you open ports 80 and 5028 (websocket).<br>

<br>
<img src="https://github.com/iw1cgw/LoRa_APRS_Meteo_dds-cgw/blob/main/img/meteo.jpg">
<br>

<br>
<img src="https://github.com/iw1cgw/LoRa_APRS_Meteo_dds-cgw/blob/main/img/dashboard.jpg">
<br>

<br>
<img src="https://github.com/iw1cgw/LoRa_APRS_Meteo_dds-cgw/blob/main/img/charts.png">
<br>
</code>
