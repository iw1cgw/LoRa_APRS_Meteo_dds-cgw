<img class="" src="https://github.com/iw1cgw/LoRa_APRS_Meteo_dds-cgw/blob/main/img/20230912.jpg?raw=true" alt="20230912.jpg" width="400" height="533" /> 
This is a personal evolution of the original OK2DDS project</p>
<p>Features: measures temperature, pressure, humidity with BMP280 or BME280 or integrated module AHT20+BMP280 or DHT22 in any combination for your meteo station, measures voltage, Ampere  with INA226 for your solar power station of any power system. Sensors are automatically recognized without setup.  The DHT22 sensor must be connected to GPIO13 with the usual 10KOhm pull-up resistor on 3.3Volt.     Check the wiring diagrams at the bottom of the page<img class="aligncenter" src="https://github.com/iw1cgw/LoRa_APRS_Meteo_dds-cgw/blob/main/img/sensor.jpg?raw=true" alt="sensor.jpg" /></p>
<p><img class="wp-image-6635 aligncenter" src="https://iw1cgw.wordpress.com/wp-content/uploads/2023/09/1.jpg" alt="1" width="375" height="744" /></p>
<p>It is possible to change the main parameters from the menu without the need to recompile again.</p>

<p>The compiled firmware can be inoculated from <strong><span style="color: #ff0000;"><a style="color: #ff0000;" href="https://iw1cgw.rf.gd/">this web page</a></span></strong> 


<p>Weather data from sensors is sent via APRS/APRS-IS LoRa.</p>
<p><img class="aligncenter" src="https://github.com/iw1cgw/LoRa_APRS_Meteo_dds-cgw/raw/main/img/aprsmap.jpg" /></p>
<p dir="auto">Before running your station please check:</p>
<ul dir="auto">
<li>you have a valid HAM radio license</li>
<li>you have edited menu configuration</li>
</ul>
<p dir="auto">For running the temperature/pressure measurement, you will need to use BMP280+AHT20 or BMP280 or BME280 sensor, sensors are automatically recognized without setup. Please solder VCC to 3.3V pin, GND to GND, SCL to IO22 and SDA to IO21. If use DTH22 sensor connect to GPIO13 ask schematic.</p>
<p dir="auto"><img class="alignnone size-full wp-image-7226 aligncenter" src="https://iw1cgw.wordpress.com/wp-content/uploads/2023/09/27a764e1-e294-42ef-b7b2-44a79ec9a70a.png" alt="" width="739" height="545" />
  
<p>The BMP280/BME280 pressure sensor does not require calibration, but the height in meters of the weather station must be set correctly from the menu to obtain a reliable SLM pressure.</p>
<p dir="auto">The detected temperature can be compensated +/- 5 Celsius from the menu; if the AHT20+BMP280 module is used, it is possible to choose which of the 2 available thermometers to use.</p>
<p dir="auto">The BMP280/BME280 pressure sensor does not require calibration, but the height in meters of the weather station must be set correctly from the menu to obtain a reliable SLM pressure.</p>
<p>If the meteostation is connected to Wi-Fi, it runs a tiny webserver.</p>
<p>Access your station dashboard with entering your station IP to your browser.</p>
<p>For accessing from outside your home network, make sure you open ports <strong><span style="color: #ff0000;">80</span></strong> and <strong><span style="color: #ff0000;">5028</span></strong> (websocket).</p>
<p>Some suggestions for solar panel powered solutions, but as per previous example you can simply connect a single BME280 or DTH22 to already have a mini weather station.</p>
<p> </p>
<p><img src="https://github.com/iw1cgw/LoRa_APRS_Meteo_dds-cgw/raw/main/img/TTGO_solar_meteo2.jpg" /></p>
<p>regulator charge for 18650 battery is: <a href="https://it.aliexpress.com/item/1005007524410968.html"><img class="alignnone size-medium wp-image-7378" src="https://iw1cgw.wordpress.com/wp-content/uploads/2023/09/18650_reg.jpg?w=300" alt="" width="300" height="234" /></a></p>
<hr />
<p><img src="https://github.com/iw1cgw/LoRa_APRS_Meteo_dds-cgw/raw/main/img/TTGO_solar_igate.jpg" /></p>
<p>step-down regulator is:<a href="https://it.aliexpress.com/item/1005005870392716.html"><img class="alignnone size-medium wp-image-7273" src="https://iw1cgw.wordpress.com/wp-content/uploads/2023/09/step_down.jpg?w=300" alt="" width="300" height="239" /></a></p>
<hr />
<p>relative Telemetry data from aprs.fi</p>
<p dir="auto"><img src="https://github.com/iw1cgw/LoRa_APRS_Meteo_dds-cgw/raw/main/img/telemetrix.jpg" /></p>
<hr />
<h3 dir="auto"> </h3>
<h3 dir="auto" style="text-align: center;"><span style="color: #ff0000;"><a href="https://drive.google.com/file/d/1Pnxnz_9v7YjMULYPtb3G8VvSXfsne7Ww/view?usp=sharing" target="_blank" rel="noopener">download firmware &amp; tools</a> -</span><a href="https://github.com/iw1cgw/LoRa_APRS_Meteo_dds-cgw" target="_blank" rel="noopener"><img class="alignnone size-full wp-image-6204" src="https://iw1cgw.files.wordpress.com/2023/09/github.jpg" alt="github" width="300" height="168" /></a></h3>
<hr />
