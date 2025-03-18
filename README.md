# LoRa_APRS_Meteo
LoRa APRS Meteostation, IGate and Digipeater firmware for TTGO LoRa32 V2.1 
this is a personal evolution of the original OK2DDS project<br><br>
Features: measures temperature, pressure, humidity with BMP280 or BME280 or AHT20 or AHT20+BMP280 or DHT22 in any combination for your meteo station, 
measures voltage, Ampere ( with INA226 ) for your solar power station of any power system.
Sensors are automatically recognized without setup.
The DHT22 sensor must be connected to GPIO13 with the usual 10KOhm pull-up resistor on 3.3Volt.
Check the wiring diagrams at the bottom of the page

<img src="https://github.com/iw1cgw/LoRa_APRS_Meteo_dds-cgw/blob/main/img/sensor.jpg">
<img src="https://github.com/iw1cgw/LoRa_APRS_Meteo_dds-cgw/blob/main/img/0.jpg">
<img src="https://github.com/iw1cgw/LoRa_APRS_Meteo_dds-cgw/blob/main/img/2.jpg">

It is possible to change the parameters from the menu without the need to recompile again.

<img src="https://github.com/iw1cgw/LoRa_APRS_Meteo_dds-cgw/blob/main/img/1.jpg">

In the 'bin' directory there is the compiled firmware which can be inoculated using the usual tools or my CGWloader.
The firmware must be inoculated starting from location 0x0000

<img src="https://github.com/iw1cgw/LoRa_APRS_Meteo_dds-cgw/blob/main/img/CGWloader.jpg">

Weather data from sensors is sent via APRS/APRS-IS LoRa. Works as a standard APRS LoRa IGate and/or digipeater, reports coverage.

<img src="https://github.com/iw1cgw/LoRa_APRS_Meteo_dds-cgw/blob/main/img/aprsmap.jpg">

Before running your station please check:
<br>
- you have a valid HAM radio license
- you have edited menu configuration
<br><br><br>


For running the temperature/pressure measurement, you will need to use BMP280+AHT20 or BMP280 or BME280 sensor, sensors are automatically recognized without setup.
Please solder VCC to 3.3V pin, GND to GND, SCL to IO22 and SDA to IO21.

<img src="https://github.com/iw1cgw/LoRa_APRS_Meteo_dds-cgw/blob/main/img/simply_bme280.jpeg">

The BMP280/BME280 pressure sensor does not require calibration, but the height in meters of the weather station must be set correctly from the menu to obtain a reliable SLM pressure.
<br><br>
The detected temperature can be compensated +/- 5 Celsius from the menu.
<br><br>
If the meteostation is connected to Wi-Fi, it runs a tiny webserver.
<br><br>
Some of the endpoints are even suitable for browser use.
<br><br>
Access your station dashboard with entering your station IP to your browser.
<br><br>
For accessing from outside your home network, make sure you open ports 80 and 5028 (websocket).
<br><br>
Some suggestions for solar panel powered solutions, but as per previous example you can simply connect a single BME280 to already have a mini weather station.

<img src="https://github.com/iw1cgw/LoRa_APRS_Meteo_dds-cgw/blob/main/img/TTGO_solar_meteo.jpg">
<br><br>
<img src="https://github.com/iw1cgw/LoRa_APRS_Meteo_dds-cgw/blob/main/img/TTGO_solar_meteo2.jpg">
<br><br>
<img src="https://github.com/iw1cgw/LoRa_APRS_Meteo_dds-cgw/blob/main/img/TTGO_solar_igate.jpg">

step-up regulator is: https://it.aliexpress.com/item/4001203401456.html
step-down regulator is: https://it.aliexpress.com/item/1005005870392716.html

relative Telemetry data from aprs.fi

<img src="https://github.com/iw1cgw/LoRa_APRS_Meteo_dds-cgw/blob/main/img/telemetrix.jpg">

If you need help or have any questions or suggestions fot this realise, please reach:iw1cgw(at)libero.it
</code>
