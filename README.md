p><img class="" src="https://github.com/iw1cgw/LoRa_APRS_Meteo_dds-cgw/blob/main/img/20230912.jpg?raw=true" alt="20230912.jpg" width="400" height="533" /><img class="" src="https://iw1cgw.files.wordpress.com/2023/04/images.jpg" alt="images" width="280" height="178" /></p>
<hr />
<p><img class="wp-image-6312 alignleft" src="https://iw1cgw.files.wordpress.com/2023/09/umarel.jpg" alt="umarel" width="346" height="346" />Hai sentito parlare dei dispositivi <strong><span style="color: #0000ff;"><a style="color: #0000ff;" href="https://it.aliexpress.com/item/32872078587.html?">TTGO</a></span></strong> che trasmettono in <strong><span style="color: #0000ff;"><a style="color: #0000ff;" href="https://it.wikipedia.org/wiki/LoRa">LoRa</a></span></strong> a 433.775MHz, la cosa ti incuriosisce ma ti sei arreso/arenato perché non sei a tuo agio con i linguaggi di programmazione dove é tutto un <em><strong>sudo</strong></em>re, un dichiarare, assemblare, compilare, seguendo indicazioni  scritte in un buon ermetico sanscrito per iniziati ? <a href="https://iw1cgw.rf.gd/" target="_blank" rel="noopener"><strong><span style="color: #0000ff;">Qui</span></strong></a> puoi facilmente installare un firmware che dovrai solo personalizzare con il tuo call, latitudine, longitudine per essere 'on-line' immediatamente e se aggiungi i <strong><span style="color: #0000ff;"><a style="color: #0000ff;" href="https://it.aliexpress.com/item/1005003975172816.html?">classici sensori di temperatura, umidità e pressione</a></span></strong> realizzi una stazione meteo minimale che puoi consultare dal tuo smartphone o meglio in rete <strong><span style="color: #0000ff;"><a style="color: #0000ff;" href="https://aprs.fi/#!call=a%2FIW1CGW-13&amp;timerange=3600&amp;tail=3600">APRS. </a></span></strong></p>
<h3 style="text-align: center;"><span style="color: #ff0000;">cosa offre questa soluzione di diverso dal 'solito' ? </span></h3>
<ul>
<li>intanto é <strong><span style="color: #0000ff;">100% compliants con le raccomandazioni di aprs.org</span></strong> quindi non potrai utilizzare pittoresche icone rosse o palle blu con la 'L' - fuori norma - [<span style="color: #000000;"> <em><strong><a style="color: #000000;" href="https://iw1cgw.wordpress.com/2024/12/03/fantasy-lora/">leggi qui per approfondimento</a></strong></em></span> ] facendo la figura di quello che non conosce le regole di aprs.org :-)</li>
<li>funzionalità di iGate e/o digipeater e stazione meteo minimale con sensori di umidità pressione temperatura e opzionalmente voltmetro/amperometro per controllo dei sistemi di alimentazione ( tipicamante pannello solare e batteria di accumulo )</li>
<li>controllo delle funzionalità del digipeater
<ul>
<li>evitando di imbarcare stazioni oltre i 30 Km</li>
<li>evitando di imbarcare il traffico del vicino di casa</li>
<li>imbarcando il traffico di una sola stazione - per particolari situazioni/necessità -</li>
<li>impossibilità, accidentale/inconsapevole, di attivare il digipeater avendo già l'iGate attivo</li>
<li>ignora richieste WIDE2-2 di altri digipeater, gestisce solo WIDE1-1</li>
<li>non gestisce stazioni che non siano 'mobili' o 'Wx remote'</li>
<li>il tutto per evitare di aumentare l'entropia di sistema [ <span style="color: #000000;"><em><strong><a style="color: #000000;" href="https://www.iz2uuf.net/wp/index.php/2016/02/17/aprs-al-collasso/">leggi qui per approfondimento</a></strong></em></span> ]</li>
</ul>
</li>
<li>genera beacon in RF non meno di ogni 10 minuti</li>
<li>genera beacon in RF che possono essere ripetuti una e una sola volta da altri digipeater</li>
<li>ignora stazioni che palesemente utilizzano call INESISTENTI, FALSI, DI FANTASIA</li>
<li>aggiornamento automatico non appena é disponibile una nuova versione</li>
<li>funzione back-up per perdita di connessione WiFi o non funzionalità internet; il controllo non si limita alla sola presenza della rete WiFi ma anche sulla piena funzionalità e risposta del server APRS-IS cui si é collegati.     Viene attivato il backup anche nei casi in cui il WiFi casalingo è presente ma per qualche disservizio del gestore di rete non c'é connettività; a più d’uno che usa connettività in 3G/4G sono capitate situazioni dove il TTGO è perfettamente connesso in WiFi al router ma proprio il router non ottiene temporaneamente nessuna connettività dalla SIM per il trasporto verso internet.</li>
<li>possibilità di installare in modalità 'auto-sense' senza necessità di configurazione gli usuali sensori: BMP280 / BME280 / AHT20 / DTH22 / INA226</li>
<li>modalità deep-sleep-mode per l'utilizzo come stazione meteo alimentata da pannello solare limitando a soli 1,8 milliAmpére i consumi nei periodi di inattività tra un invio dati e l'altro [ vedi a fondo pagina alcune suggestioni per rrealizzare una stazione meteo solar/powered ]</li>
<li>supporto per l'invio dei dati meteo sulla piattaforma <span style="color: #000000;"><em><strong><a style="color: #000000;" href="https://www.wunderground.com/">Wunder</a></strong></em></span></li>
<li><span style="font-size: revert;">Nel funzionamento come iGate il sistema offre nel 'fumetto' l'indicazione - </span><span style="color: #ff00ff;">carattere fucsia</span><span style="font-size: revert;"> - dell'ultimo corrispondente ascoltato con i valori di SNR in dB e di livello segnale RF in dBm; il tutto può essere interessante per analisi di aspetti 'propagativi' o prestazionali dei sistemi LoRa tech. Nell'esempio ho ricevuto da circa 35 Km di distanza la stazione IZ1BLA-20 con segnale di -117dbm con (S)ignal(N)oise(R)atio di -11.5 dB.</span></li>
</ul>
<p><img class="size-full wp-image-6301 aligncenter" src="https://iw1cgw.files.wordpress.com/2023/09/bla.jpg" alt="bla" width="764" height="455" /></p>
<hr />
<p><a href="https://iw1cgw.rf.gd/"><img class="size-full wp-image-7296 aligncenter" src="https://iw1cgw.wordpress.com/wp-content/uploads/2023/09/wf.jpg" alt="" width="729" height="583" /></a></p>
<hr />
<p>Al primo avvio troverai sulla rete WiFi un access-point 'N0CALL-10' a cui potrai collegarti con la password '00000000' - 8 volte 'zero e da qui iniziare il setup.  Se non ti sono chiari i parametri da inserire consulta il</p>
<h1 style="text-align: center;"><span style="color: #ff0000;"><a style="color: #ff0000;" href="https://iw1cgw.wordpress.com/lora-aprs-meteoigate/">&gt;&gt;&gt; manuale &lt;&lt;&lt;</a></span></h1>
<hr />
<p style="text-align: left;">This is a personal evolution of the original OK2DDS project</p>
<p>Features: measures temperature, pressure, humidity with BMP280 or BME280 or integrated module AHT20+BMP280 or DHT22 in any combination for your meteo station, measures voltage, Ampere  with INA226 for your solar power station of any power system. Sensors are automatically recognized without setup.  The DHT22 sensor must be connected to GPIO13 with the usual 10KOhm pull-up resistor on 3.3Volt.     Check the wiring diagrams at the bottom of the page<img class="aligncenter" src="https://github.com/iw1cgw/LoRa_APRS_Meteo_dds-cgw/blob/main/img/sensor.jpg?raw=true" alt="sensor.jpg" /></p>
<p><img class="wp-image-6635 aligncenter" src="https://iw1cgw.wordpress.com/wp-content/uploads/2023/09/1.jpg" alt="1" width="375" height="744" /></p>
<p>It is possible to change the main parameters from the menu without the need to recompile again.</p>
<p><img class="wp-image-6634 aligncenter" src="https://iw1cgw.wordpress.com/wp-content/uploads/2023/09/0.jpg" alt="0" width="375" height="743" /></p>
<p>The compiled firmware can be inoculated from <strong><span style="color: #ff0000;"><a style="color: #ff0000;" href="https://iw1cgw.rf.gd/">this web page</a></span></strong> or with my CGW_loader. The firmware must be inoculated starting from position 0x0000 and you can get it on <strong><span style="color: #ff0000;"><a style="color: #ff0000;" href="https://github.com/iw1cgw/LoRa_APRS_Meteo_dds-cgw">GitHub</a></span></strong>.</p>
<p><img class="aligncenter" src="https://github.com/iw1cgw/LoRa_APRS_Meteo_dds-cgw/raw/main/img/CGWloader.jpg" /></p>
<p>Weather data from sensors is sent via APRS/APRS-IS LoRa.</p>
<p><img class="aligncenter" src="https://github.com/iw1cgw/LoRa_APRS_Meteo_dds-cgw/raw/main/img/aprsmap.jpg" /></p>
<p dir="auto">Before running your station please check:</p>
<ul dir="auto">
<li>you have a valid HAM radio license</li>
<li>you have edited menu configuration</li>
</ul>
<p dir="auto">For running the temperature/pressure measurement, you will need to use BMP280+AHT20 or BMP280 or BME280 sensor, sensors are automatically recognized without setup. Please solder VCC to 3.3V pin, GND to GND, SCL to IO22 and SDA to IO21. If use DTH22 sensor connect to GPIO13 ask schematic.</p>
<p dir="auto"><img class="alignnone size-full wp-image-7226 aligncenter" src="https://iw1cgw.wordpress.com/wp-content/uploads/2023/09/27a764e1-e294-42ef-b7b2-44a79ec9a70a.png" alt="" width="739" height="545" />The BMP280/BME280 pressure sensor does not require calibration, but the height in meters of the weather station must be set correctly from the menu to obtain a reliable SLM pressure.</p>
<p dir="auto">The detected temperature can be compensated +/- 5 Celsius from the menu; if the AHT20+BMP280 module is used, it is possible to choose which of the 2 available thermometers to use.</p>
<p dir="auto">The BMP280/BME280 pressure sensor does not require calibration, but the height in meters of the weather station must be set correctly from the menu to obtain a reliable SLM pressure.</p>
<p>If the meteostation is connected to Wi-Fi, it runs a tiny webserver.</p>
<p>Access your station dashboard with entering your station IP to your browser.</p>
<p>For accessing from outside your home network, make sure you open ports <strong><span style="color: #ff0000;">80</span></strong> and <strong><span style="color: #ff0000;">5028</span></strong> (websocket).</p>
<p>Some suggestions for solar panel powered solutions, but as per previous example you can simply connect a single BME280 or DTH22 to already have a mini weather station.</p>
<p><img src="https://github.com/iw1cgw/LoRa_APRS_Meteo_dds-cgw/raw/main/img/TTGO_solar_meteo1.jpg" /></p>
<p>step-up regulator with battery charger is: <a href="https://it.aliexpress.com/item/4001203401456.html"><img class="alignnone size-medium wp-image-7266" src="https://iw1cgw.wordpress.com/wp-content/uploads/2023/09/step_up_regulator.jpg?w=300" alt="" width="300" height="204" /></a></p>
<hr />
<p><img src="https://github.com/iw1cgw/LoRa_APRS_Meteo_dds-cgw/raw/main/img/TTGO_solar_meteo2.jpg" /></p>
<p>step-down regulator is: <a href="https://it.aliexpress.com/item/1005005870392716.html"><img class="alignnone size-medium wp-image-7273" src="https://iw1cgw.wordpress.com/wp-content/uploads/2023/09/step_down.jpg?w=300" alt="" width="300" height="239" /></a></p>
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
