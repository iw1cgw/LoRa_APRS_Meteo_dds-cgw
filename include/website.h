#define webPageStart "<!DOCTYPE html><html><head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\"><style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;} body { height: 99%;} footer { color: #696969; width: 100%; text-align: center;} a { color: #696969; font-family: Helvetica;} a:visited { color: #696969;} a:hover { color: #002fba;} table { width: 100%; text-align: center; margin-left: auto; margin-right: auto; table-layout: fixed;} .tr { line-height: 300%;} .header { font-size: 12pt;} .watchheader { font-size: 26pt;} .value { font-weight: bold; font-size: 16pt;} .watchvalue { font-weight: bold; font-size: 28pt;} b { padding: 7px; border-radius: 5px;} .good { color: white; background: #00702f;} .bad { color: white; background: red;} .cold { color: #004594;} .hot { color: #940000;} .charged { color: #00702f;}</style><title>LoRa Meteo</title></head><body>"
#define webPageHeader "<h1>LoRa Meteostation"
//#define webPageFooter "<br><br><footer>Created by <a href=\"https://www.ok2dds.cz\">OK2DDS</a> - Modified by <a href=\"https://iw1cgw.wordpress.com\">IW1CGW</a><br><br>   <a href=\"/graphs\">Weather charts</a><br><a href=\"/lora\">View dashboard</a><br><br>Created by <a href=\"https://www.ok2dds.cz\">OK2DDS</a> - Modified by <a href=\"https://iw1cgw.wordpress.com\">IW1CGW</a></footer>"
#define webPageFooter "<br><br><br><footer><a href='/'>View main meteo page</a> <br> <a href=\"/graphs\">Weather charts</a> <br> <a href=\"/lora\">View dashboard</a><br><br>Created by <a href=\"https://www.ok2dds.cz\">OK2DDS</a> - Modified by <a href=\"https://iw1cgw.wordpress.com\">IW1CGW</a></footer>"
#define webPageEnd "</body></html>"
#define webReload "<script>window.location.href = '/lora';</script>"
#define webAPRSISChangePrompt "<script>window.location.href = '/new-aprsis/' + prompt('Enter new APRS-IS server address', null);</script>"
#define webAPRSISChangeError "<script>alert('Error setting new APRS-IS server'); window.location.href = '/lora';</script>"
#define webAPRSISChangeSuccess "<script>alert('Connected to new APRS-IS server'); setTimeout(window.location.href = '/lora', 4500);</script>"

#define webMeteoOnlineIndicator "<b id=\"onlineIndicator\" class=\"bad\">Meteostation offline</b><br>"
#define webMeteoLayout "<table><tr class=\"tr\"><td class=\"header\">Temperature</td><td class=\"header\">Hum</td><td class=\"header\">Pressure</td></tr><tr class=\"tr\"><td class=\"value\" id=\"temp\">N/A</td><td class=\"value\" id=\"Hum\">N/A</td><td class=\"value\" id=\"press\">N/A</td></tr><tr class=\"tr\"></tr></table>"
#define webSocketSetupScript "<script>var ip = location.host.split(':')[0]; const ws = new WebSocket('ws://' + ip + ':5028/ws'); ws.onopen = function() { console.log('WS: Connection opened');}; ws.onclose = function() { console.log('WS: Connection closed'); };"
#define webMeteoOnlineRoutine "var lastData = 0; function check() { if (lastData + 6500 < Date.now()) { onlineIndicator.innerHTML = 'Meteostation offline'; onlineIndicator.className = 'bad';} else { onlineIndicator.innerHTML = 'Meteostation online'; onlineIndicator.className = 'good';}} check(); var timeOut = setInterval(check, 1000);"
#define webSocketHandleScript "ws.onmessage = function(event) { var dat = event.data; var bmp = true; var anemo = true; if (dat.split(',')[0] == 'N/A' && dat.split(',')[2] == 'N/A') { bmp = false;} if (dat.split(',')[3] == 'N/A' && dat.split(',')[4] == 'N/A' && dat.split(',')[5] == 'N/A' && dat.split(',')[6] == 'N/A') { anemo = false;} if (bmp) { temp.innerHTML = dat.split(',')[0] + ' &deg;C'; Hum.innerHTML = dat.split(',')[1] + ' %' ; press.innerHTML = dat.split(',')[2] + ' hPa';} if (bmp && dat.split(',')[0] > 27) { temp.className = 'value hot';} else if (bmp && dat.split(',')[0] < 15) { temp.className = 'value cold';} else { temp.className = 'value';} lastData = Date.now(); };</script>"

#define webMeteoWatchLayout "<table style='height: 50%; position: absolute'><tr><td class=\"watchheader\">Temperature</td><td class=\"watchheader\">Pressure</td></tr><tr><td class=\"watchvalue\" id=\"temp\">N/A</td><td class=\"watchvalue\" id=\"press\">N/A</td></tr><tr><br><footer style='position: absolute; bottom: 45%'>Created by OK2DDS modified by IW1CGW.</footer>"
#define webWatchValuesScript " temp.innerHTML = dat.split(',')[0] + ' &deg;C'; Hum.innerHTML = dat.split(',')[1] + ' hum'; press.innerHTML = dat.split(',')[2] + ' hPa'; if (dat.split(',')[0] > 27) { temp.className = 'watchvalue hot';} else if (dat.split(',')[0] < 15) { temp.className = 'watchvalue cold';} else { temp.className = 'watchvalue';}</script>"