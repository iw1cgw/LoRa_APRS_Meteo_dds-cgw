#define webPageStart "<!DOCTYPE html><html><head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\"><style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;} body { height: 99%;} footer { color: #696969; width: 100%; text-align: center;} a { color: #696969; font-family: Helvetica;} a:visited { color: #696969;} a:hover { color: #002fba;} table { width: 100%; text-align: center; margin-left: auto; margin-right: auto; table-layout: fixed;} .tr { line-height: 300%;} .header { font-size: 12pt;} .watchheader { font-size: 26pt;} .value { font-weight: bold; font-size: 16pt;} .watchvalue { font-weight: bold; font-size: 28pt;} b { padding: 7px; border-radius: 5px;} .good { color: white; background: #00702f;} .bad { color: white; background: red;} .cold { color: #004594;} .hot { color: #940000;} .charged { color: #00702f;}</style><title>LoRa Meteo</title></head><body>"
#define webPageHeader "<h1>LoRa Meteostation"
//#define webPageFooter "<br><br><footer>Created by <a href=\"https://www.ok2dds.cz\">OK2DDS</a> - Modified by <a href=\"https://iw1cgw.wordpress.com\">IW1CGW</a><br><br>   <a href=\"/graphs\">Weather charts</a><br><a href=\"/lora\">View dashboard</a><br><br>Created by <a href=\"https://www.ok2dds.cz\">OK2DDS</a> - Modified by <a href=\"https://iw1cgw.wordpress.com\">IW1CGW</a></footer>"
#define webPageFooter "<br><br><br><footer><a href='/'>view main meteo page</a> <br> <a href=\"/graphs\">weather charts</a> <br> <a href=\"/lora\">view dashboard</a><br><br>Created by <a href=\"https://www.ok2dds.cz\">OK2DDS</a> - Modified by <a href=\"https://iw1cgw.wordpress.com\">IW1CGW</a></footer>"
#define webPageEnd "</body></html>"
#define webReload "<script>window.location.href = '/lora';</script>"
#define webTest "<script>window.location.href = '/test';</script>"

#define web_ChangePrompt_igate "<script>window.location.href = '/new-value/' + prompt('Enter new text for iGate beacon', null);</script>"
#define web_ChangePrompt_meteo "<script>window.location.href = '/new-value/' + prompt('Enter new text for meteo beacon', null);</script>"
#define web_ChangeError "<script>alert('Error setting); window.location.href = '/lora';</script>"

#define web_ChangePrompt_temp "<script>window.location.href = '/new-value/' + prompt('new value thermal drift (max +/- 5) | ex: -0.25', null);</script>"
#define web_ChangePrompt_pres "<script>window.location.href = '/new-value/' + prompt('new value pressure drift (max +/- 10) | ex: -2', null);</script>"

#define web_ChangePrompt_power "<script>window.location.href = '/new-value/' + prompt('new value power (max 20 dbm) | ex: 2', null);</script>"

//#define webAPRSISChangePrompt "<script>window.location.href = '/new-aprsis/' + prompt('Enter new APRS-IS server address', null);</script>"
//#define webAPRSISChangeError "<script>alert('Error setting new APRS-IS server'); window.location.href = '/lora';</script>"
//#define webAPRSISChangeSuccess "<script>alert('Connected to new APRS-IS server'); setTimeout(window.location.href = '/lora', 4500);</script>"

#define webMeteoOnlineIndicator "<b id=\"onlineIndicator\" class=\"bad\">Meteostation offline</b><br>"
#define webMeteoLayout "<table><tr class=\"tr\"><td class=\"header\">Temperature</td><td class=\"header\">Hum</td><td class=\"header\">Pressure</td></tr><tr class=\"tr\"><td class=\"value\" id=\"temp\">N/A</td><td class=\"value\" id=\"Hum\">N/A</td><td class=\"value\" id=\"press\">N/A</td></tr><tr class=\"tr\"></tr></table>"
#define webSocketSetupScript "<script>var ip = location.host.split(':')[0]; const ws = new WebSocket('ws://' + ip + ':5028/ws'); ws.onopen = function() { console.log('WS: Connection opened');}; ws.onclose = function() { console.log('WS: Connection closed'); };"
#define webMeteoOnlineRoutine "var lastData = 0; function check() { if (lastData + 6500 < Date.now()) { onlineIndicator.innerHTML = 'Meteostation offline'; onlineIndicator.className = 'bad';} else { onlineIndicator.innerHTML = 'Meteostation online'; onlineIndicator.className = 'good';}} check(); var timeOut = setInterval(check, 1000);"

#define webSocketHandleScript    "ws.onmessage = function(event) { var dat = event.data; var bmp = true; var anemo = true; if (dat.split(',')[0] == 'N/A' && dat.split(',')[2] == 'N/A') { bmp = false;} if (dat.split(',')[3] == 'N/A' && dat.split(',')[4] == 'N/A' && dat.split(',')[5] == 'N/A' && dat.split(',')[6] == 'N/A') { anemo = false;} if (bmp) { temp.innerHTML = dat.split(',')[0] + ' &deg;C'; Hum.innerHTML = dat.split(',')[1] + ' %' ; press.innerHTML = dat.split(',')[2] + ' hPa';} if (bmp && dat.split(',')[0] > 27) { temp.className = 'value hot';} else if (bmp && dat.split(',')[0] < 15) { temp.className = 'value cold';} else { temp.className = 'value';} lastData = Date.now(); };</script>"
#define webSocketHandleScript_RU "ws.onmessage = function(event) { var dat = event.data; var bmp = true; var anemo = true; if (dat.split(',')[0] == 'N/A' && dat.split(',')[2] == 'N/A') { bmp = false;} if (dat.split(',')[3] == 'N/A' && dat.split(',')[4] == 'N/A' && dat.split(',')[5] == 'N/A' && dat.split(',')[6] == 'N/A') { anemo = false;} if (bmp) { temp.innerHTML = dat.split(',')[0] + ' &deg;C'; Hum.innerHTML = dat.split(',')[1] + ' %' ; press.innerHTML = dat.split(',')[2] + ' mmHg';} if (bmp && dat.split(',')[0] > 27) { temp.className = 'value hot';} else if (bmp && dat.split(',')[0] < 15) { temp.className = 'value cold';} else { temp.className = 'value';} lastData = Date.now(); };</script>"



#define webMeteoWatchLayout "<table style='height: 50%; position: absolute'><tr><td class=\"watchheader\">Temperature</td><td class=\"watchheader\">Pressure</td></tr><tr><td class=\"watchvalue\" id=\"temp\">N/A</td><td class=\"watchvalue\" id=\"press\">N/A</td></tr><tr><br><footer style='position: absolute; bottom: 45%'>Created by OK2DDS modified by IW1CGW.</footer>"

#define webWatchValuesScript    " temp.innerHTML = dat.split(',')[0] + ' &deg;C'; Hum.innerHTML = dat.split(',')[1] + ' hum'; press.innerHTML = dat.split(',')[2] + ' hPa'; if (dat.split(',')[0] > 27) { temp.className = 'watchvalue hot';} else if (dat.split(',')[0] < 15) { temp.className = 'watchvalue cold';} else { temp.className = 'watchvalue';}</script>"
#define webWatchValuesScript_RU " temp.innerHTML = dat.split(',')[0] + ' &deg;C'; Hum.innerHTML = dat.split(',')[1] + ' hum'; press.innerHTML = dat.split(',')[2] + ' mmHg'; if (dat.split(',')[0] > 27) { temp.className = 'watchvalue hot';} else if (dat.split(',')[0] < 15) { temp.className = 'watchvalue cold';} else { temp.className = 'watchvalue';}</script>"


/*

{



  serverBB.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });

  serverBB.on("/get", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String input_message;
    String input_parameter;

    if (request->hasParam(input_parameter1)) {
      input_message = request->getParam(input_parameter1)->value();
      input_parameter = input_parameter1;
    }
    else if (request->hasParam(input_parameter2)) {
      input_message = request->getParam(input_parameter2)->value();
      input_parameter = input_parameter2;
    }

    else if (request->hasParam(input_parameter3)) {
      input_message = request->getParam(input_parameter3)->value();
      input_parameter = input_parameter3;
    }
    else {
      input_message = "No message sent";
      input_parameter = "none";
    }
    Serial.println(input_message);
    request->send(200, "text/html", "HTTP GET request sent to your ESP on input field ("+ input_parameter + ") with value: " + input_message + "<br><a href=\"/\">Return to Home Page</a>");
  });
    serverBB.onNotFound(notFound);
  serverBB.begin();
}
*/
