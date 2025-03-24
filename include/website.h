//#define webPageStart "<!DOCTYPE html><html><head>        <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">  <style> body{ margin: 10px, padding: 0;font-family: Arial, Helvetica, sans-serif;color: white; background-color: #2c257a;}  fieldset {  width:400px; border:2px solid #22a6b3; border-radius:12px; box-shadow:0 0 10px #22a6b3;}                                                                                   a { color: #696969; font-family: Helvetica;} a:visited { color: #696969;} a:hover { color: #002fba;} table { width: 100%; text-align: center; margin-left: auto; margin-right: auto; table-layout: fixed;} .tr { line-height: 300%;} .header { font-size: 13pt;} .watchheader { font-size: 13pt;} .value { font-weight: bold; font-size: 13pt;} .watchvalue { font-weight: bold; font-size: 13pt;} b { padding: 7px; border-radius: 5px;} .good { color: white; background: #00702f;} .bad { color: white; background: red;} .cold { color: #004594;} .hot { color: #940000;} .charged { color: #00702f;}</style> </head><body>"


#define webPageStart "<!DOCTYPE html><html><head>   <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">    <style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}       fs9 { text-align: left;font-size: 9pt}     fs10 { font-size: 10pt; }   fs12 { font-size: 12pt;}  fs13 { font-size: 13pt;}  fs14 { font-size: 14pt;}   fs16 { font-size: 16pt;}        fieldset {  font-size: 12pt; width:350px; border:2px solid #22a6b3; border-radius:12px; box-shadow:0 0 10px #22a6b3;}   body { height: 99%;} footer { color: #696969; width: 100%; text-align: center;} a { color: #696969; font-family: Helvetica;} a:visited { color: #696969;} a:hover { color: #002fba;} table { width: 100%; text-align: center; margin-left: auto; margin-right: auto; table-layout: fixed;} .tr { line-height: 300%;} .header { font-size: 12pt;} .watchheader { font-size: 12pt;} .value { font-weight: bold; font-size: 12pt;} .watchvalue { font-weight: bold; font-size: 12pt;} b { padding: 7px; border-radius: 5px;} .good { color: white; background: #00702f;} .bad { color: white; background: red;} .cold { color: #004594;} .hot { color: #940000;} .charged { color: #00702f;}</style><title>LoRa Meteo</title></head><body>"


//#define webPageHeader "<h1>LoRa digi iGate meteo"
//#define webPageFooter "<br><br><footer>Created by <a href=\"https://www.ok2dds.cz\">OK2DDS</a> - Modified by <a href=\"https://iw1cgw.wordpress.com\">IW1CGW</a><br><br>   <a href=\"/graphs\">Weather charts</a><br><a href=\"/lora\">View dashboard</a><br><br>Created by <a href=\"https://www.ok2dds.cz\">OK2DDS</a> - Modified by <a href=\"https://iw1cgw.wordpress.com\">IW1CGW</a></footer>"
//#define webPageFooter "<footer><a href='/'>view main meteo page</a> <br> <a href=\"/graphs\">weather charts</a> <br> <a href=\"/lora\">view dashboard</a> - <a href=\"https://iw1cgw.wordpress.com/lora-aprs-meteoigate\">user manual</a><br><br>by <a href=\"https://iw1cgw.wordpress.com\">IW1CGW</a> based on <a href=\"https://www.ok2dds.cz\">OK2DDS</a> 'project</footer>"
#define webPageFooter_ip "<footer><a href=\"/graphs\">weather charts</a> - <a href=\"https://iw1cgw.wordpress.com/lora-aprs-meteoigate\">user manual</a> - <a href=\"/ \">dashboard</a></a><br><br>by <a href=\"https://iw1cgw.wordpress.com\">IW1CGW</a> inspired by <a href=\"https://www.ok2dds.cz\">OK2DDS</a> 'project</footer>"
#define webPageFooter_noip "<footer><a href=\"/ \">dashboard</a></a><br><br>by IW1CGW inspired by OK2DDS project</footer>"

//#define webPageFooter_noip "<footer><a href=\"https://iw1cgw.wordpress.com/lora-aprs-meteoigate\">user manual</a><br><br>by <a href=\"https://iw1cgw.wordpress.com\">IW1CGW</a> based on <a href=\"https://www.ok2dds.cz\">OK2DDS</a> 'project</footer>"
//#define webPageFooter "<footer><a href=\"/graphs\">weather charts</a> - <a href=\"https://iw1cgw.wordpress.com/lora-aprs-meteoigate\">user manual</a><br><br>by <a href=\"https://iw1cgw.wordpress.com\">IW1CGW</a> based on <a href=\"https://www.ok2dds.cz\">OK2DDS</a> 'project</footer>"
#define webPageEnd "</body></html>"
#define webReload "<script>window.location.href = '/';</script>"
#define setupReload "<script>window.location.href = '/setup';</script>"
#define loginReload "<script>window.location.href = '/login';</script>"

#define web_ChangePrompt_value "<script>window.location.href = '/new-value/' + prompt('enter value', null);</script>"
#define web_ChangePrompt_text "<script>window.location.href = '/new-value/' + prompt('enter text', null);</script>"
#define web_ChangeError "<script>alert('Error setting); window.location.href = '/';</script>"

#define web_ChangePrompt_temp "<script>window.location.href = '/new-value/' + prompt('thermal drift (max +/- 5) | ex: -0.25', null);</script>"
#define web_ChangePrompt_pres "<script>window.location.href = '/new-value/' + prompt('pressure drift (max +/- 10) | ex: -2', null);</script>"
#define web_ChangePrompt_vw "<script>window.location.href = '/new-value/' + prompt('weathervane drift (max +/- 125) | ex: -10', null);</script>"
#define web_ChangePrompt_anemo "<script>window.location.href = '/new-value/' + prompt('anemometer circumference) | ex: 14.95', null);</script>"
#define web_ChangePrompt_power  "<script>window.location.href = '/new-value/' + prompt('power (max 20 dbm) | ex: 2', null);</script>"
//#define web_ChangePrompt_rx_gain  "<script>window.location.href = '/new-value/' + prompt('rf gain range 1 to 6 where 1 is the highest gain. Set to 0 to enable automatic gain control (recommended) | ex: 0 for AUTO', null);</script>"

#define web_ChangePrompt_frequency "<script>window.location.href = '/new-value/' + prompt('frequency in KHz | ex: 433775', null);</script>"
#define web_ChangePrompt_latlong "<script>window.location.href = '/new-value/' + prompt('lat,long - ex: 78.7562,18.5162', null);</script>"
#define web_ChangePrompt_beacon_interval  "<script>window.location.href = '/new-value/' + prompt('beacons tx interval (min 10 min - 0 to disable | ex: 20', null);</script>"
#define web_ChangePrompt_SSid  "<script>window.location.href = '/new-value/' + prompt('SSiD value (max 99 or empty | ex: 10', null);</script>"
#define web_ChangePrompt_pwd  "<script>window.location.href = '/new-value/' + prompt('enter password', null);</script>"
#define web_ChangePrompt_radius "<script>window.location.href = '/new-value/' + prompt('Km digipeater radius (max 255 - 0 disable function) | ex: 12', null);</script>"
#define web_ChangePrompt_route "<script>window.location.href = '/new-value/' + prompt('digipeat route is ONLY for: | ex: IR1XYZ', null);</script>"
#define web_ChangePrompt_banned "<script>window.location.href = '/new-value/' + prompt('digipeat route denied for: | ex: IR1XYZ', null);</script>"

//#define webAPRSISChangePrompt "<script>window.location.href = '/new-aprsis/' + prompt('Enter new APRS-IS server address', null);</script>"
//#define webAPRSISChangeError "<script>alert('Error setting new APRS-IS server'); window.location.href = '/lora';</script>"
//#define webAPRSISChangeSuccess "<script>alert('Connected to new APRS-IS server'); setTimeout(window.location.href = '/lora', 4500);</script>"

#define webMeteoOnlineIndicator "<b id=\"onlineIndicator\" class=\"bad\">Meteostation offline</b><br>"
#define webMeteoLayout     "<table><tr class=\"tr\"> <td class=\"header\">Temp.</td> <td class=\"header\">DewPoint</td> <td class=\"header\">Hum</td> <td class=\"header\">Pressure</td></tr><tr class=\"tr\"> <td class=\"value\" id=\"temp\">N/A</td> <td class=\"value\" id=\"DewPoint\">N/A</td> <td class=\"value\" id=\"Hum\">N/A</td> <td class=\"value\" id=\"press\">N/A</td> </tr><tr class=\"tr\"></tr></table>"
#define webMeteoLayout_BMP "<table><tr class=\"tr\"> <td class=\"header\">Temp.</td>                                                                  <td class=\"header\">Pressure</td></tr><tr class=\"tr\"> <td class=\"value\" id=\"temp\">N/A</td>                                                                                      <td class=\"value\" id=\"press\">N/A</td> </tr><tr class=\"tr\"></tr></table>"
#define webMeteoLayout_DHT "<table><tr class=\"tr\"> <td class=\"header\">Temp.</td> <td class=\"header\">DewPoint</td> <td class=\"header\">Hum</td>                                   </tr><tr class=\"tr\"> <td class=\"value\" id=\"temp\">N/A</td> <td class=\"value\" id=\"DewPoint\">N/A</td> <td class=\"value\" id=\"Hum\">N/A</td>                                           </tr><tr class=\"tr\"></tr></table>"

#define webSocketSetupScript "<script>var ip = location.host.split(':')[0]; const ws = new WebSocket('ws://' + ip + ':5028/ws'); ws.onopen = function() { console.log('WS: Connection opened');}; ws.onclose = function() { console.log('WS: Connection closed'); };"
#define webMeteoOnlineRoutine "var lastData = 0; function check() { if (lastData + 6500 < Date.now()) { onlineIndicator.innerHTML = 'Meteostation offline'; onlineIndicator.className = 'bad';} else { onlineIndicator.innerHTML = 'Meteostation online'; onlineIndicator.className = 'good';}} check(); var timeOut = setInterval(check, 1000);"

#define webSocketHandleScript     "ws.onmessage = function(event) { var dat = event.data; var bmp = true; var anemo = true; if (dat.split(',')[0] == 'N/A' && dat.split(',')[2] == 'N/A') { bmp = false;} if (dat.split(',')[3] == 'N/A' && dat.split(',')[4] == 'N/A' && dat.split(',')[5] == 'N/A' && dat.split(',')[6] == 'N/A') { anemo = false;} if (bmp) { temp.innerHTML = dat.split(',')[0] + ' &deg;C'; DewPoint.innerHTML = dat.split(',')[1] + ' &deg;C'; Hum.innerHTML = dat.split(',')[2] + ' %' ; press.innerHTML = dat.split(',')[3] + ' hPa'; } if (bmp && dat.split(',')[0] > 27) { temp.className = 'value hot';} else if (bmp && dat.split(',')[0] < 15) { temp.className = 'value cold';} else { temp.className = 'value';} lastData = Date.now(); };</script>"
#define webSocketHandleScript_RU  "ws.onmessage = function(event) { var dat = event.data; var bmp = true; var anemo = true; if (dat.split(',')[0] == 'N/A' && dat.split(',')[2] == 'N/A') { bmp = false;} if (dat.split(',')[3] == 'N/A' && dat.split(',')[4] == 'N/A' && dat.split(',')[5] == 'N/A' && dat.split(',')[6] == 'N/A') { anemo = false;} if (bmp) { temp.innerHTML = dat.split(',')[0] + ' &deg;C'; DewPoint.innerHTML = dat.split(',')[1] + ' &deg;C'; Hum.innerHTML = dat.split(',')[2] + ' %' ; press.innerHTML = dat.split(',')[3] + ' mmHg';} if (bmp && dat.split(',')[0] > 27) { temp.className = 'value hot';} else if (bmp && dat.split(',')[0] < 15) { temp.className = 'value cold';} else { temp.className = 'value';} lastData = Date.now(); };</script>"
#define webSocketHandleScript_BMP "ws.onmessage = function(event) { var dat = event.data; var bmp = true; var anemo = true; if (dat.split(',')[0] == 'N/A' && dat.split(',')[2] == 'N/A') { bmp = false;} if (dat.split(',')[3] == 'N/A' && dat.split(',')[4] == 'N/A' && dat.split(',')[5] == 'N/A' && dat.split(',')[6] == 'N/A') { anemo = false;} if (bmp) { temp.innerHTML = dat.split(',')[0] + ' &deg;C';                                                                                                press.innerHTML = dat.split(',')[1] + ' hPa'; } if (bmp && dat.split(',')[0] > 27) { temp.className = 'value hot';} else if (bmp && dat.split(',')[0] < 15) { temp.className = 'value cold';} else { temp.className = 'value';} lastData = Date.now(); };</script>"
#define webSocketHandleScript_DHT "ws.onmessage = function(event) { var dat = event.data; var bmp = true; var anemo = true; if (dat.split(',')[0] == 'N/A' && dat.split(',')[2] == 'N/A') { bmp = false;} if (dat.split(',')[3] == 'N/A' && dat.split(',')[4] == 'N/A' && dat.split(',')[5] == 'N/A' && dat.split(',')[6] == 'N/A') { anemo = false;} if (bmp) { temp.innerHTML = dat.split(',')[0] + ' &deg;C'; DewPoint.innerHTML = dat.split(',')[1] + ' &deg;C'; Hum.innerHTML = dat.split(',')[2] + ' %' ;                                               } if (bmp && dat.split(',')[0] > 27) { temp.className = 'value hot';} else if (bmp && dat.split(',')[0] < 15) { temp.className = 'value cold';} else { temp.className = 'value';} lastData = Date.now(); };</script>"

#define webMeteoWatchLayout "<table style='height: 50%; position: absolute'><tr><td class=\"watchheader\">Temp.</td><td class=\"watchheader\">Pressure</td></tr><tr><td class=\"watchvalue\" id=\"temp\">N/A</td><td class=\"watchvalue\" id=\"press\">N/A</td></tr><tr><br><footer style='position: absolute; bottom: 45%'>by IW1CGW inspired by OK2DDS' project.</footer>"

#define webWatchValuesScript    " temp.innerHTML = dat.split(',')[0] + ' &deg;C'; DewPoint.innerHTML = dat.split(',')[1] + ' hum'; Hum.innerHTML = dat.split(',')[2] + ' hum'; press.innerHTML = dat.split(',')[3] + ' hPa';  if (dat.split(',')[0] > 27) { temp.className = 'watchvalue hot';} else if (dat.split(',')[0] < 15) { temp.className = 'watchvalue cold';} else { temp.className = 'watchvalue';}</script>"
#define webWatchValuesScript_RU " temp.innerHTML = dat.split(',')[0] + ' &deg;C'; DewPoint.innerHTML = dat.split(',')[1] + ' hum'; Hum.innerHTML = dat.split(',')[1] + ' hum'; press.innerHTML = dat.split(',')[2] + ' mmHg'; if (dat.split(',')[0] > 27) { temp.className = 'watchvalue hot';} else if (dat.split(',')[0] < 15) { temp.className = 'watchvalue cold';} else { temp.className = 'watchvalue';}</script>"