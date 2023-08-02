#include <Arduino.h>
#include "config.h"
#include <LoRa.h>
#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_BME280.h>
#include <Adafruit_AHTX0.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "website.h"
#include <EEPROM.h>//https://github.com/espressif/arduino-esp32/tree/master/libraries/EEPROM

void lora_setup();
void lora_send(String tx_data);
void aprsis_connect();
void upload_data(String upload_data);
void aprsis_send(String aprsis_packet);
void beacon_igate();
void beacon_meteo();
void beacon_meteo_status();
void beacon_upload();

bool check_wifi();
bool check_aprsis();

WiFiServer server(80);
WiFiClient aprsis;
HTTPClient upload;
String APRSISServer;
int pktIndex;
int mpktIndex;
int apktIndex;
AsyncWebServer serverWS(5028);
AsyncWebSocket ws("/ws");
void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len);
void updateWebSocket();
String HTMLelementDef(String elementID);
String ipToString(IPAddress ip);
bool isWSconnected = false;
unsigned long lastWSupdate = 0;
String myIP;
bool GETIndex(String header, String requestPath);

unsigned long lastDigipeat = 0;
String lastRXstation = "no station";

float voltage = 0;
int battPercent = 0;
unsigned long  millis_led = 0;
const byte PLED1 = 25;   

bool wifiStatus = false;
static time_t aprsLastReconnect = 0;
static time_t lastIgBeacon = 0;
static time_t lastMtBeacon = 0;
static time_t lastUpload = 0;

Adafruit_BME280 bme;
Adafruit_BMP280 bmp;
Adafruit_AHTX0 aht;
bool BM_sensor_status = false;
bool AHTstatus = false;
bool BMPstatus = false;
bool BMEstatus = false;

bool getBM_sensor_status();
bool getAHTstatus();
bool getBMEstatus();

float getTempC();
String getTempAPRS();

float getHum();
String getHumAPRS();

float getPressure();
String getPressureAPRS();

String tempToWeb(float tempValue);
String pressToWeb(float pressValue);
String HumToWeb(float HumValue);
String windToWeb(float windValue);
String valueForJSON(String value);

String tempValues;
String pressValues;
String HumValues;
String windValues;
float minTemp = -1000;
float maxTemp;

float minPress = -1000;
float maxPress;
float maxWind;
float maxGust;
String addGraphValue(String values, String value);
String generateGraph(String values, String graphName, String graphID, int r, int g, int b);

void hall_change();
float mph(float metersPerSeconds);
String windSpeedAPRS(float fSpeed);
int anemoACValue;
bool magnetDetected = false;
int windMeterSpins = 0;
int windMeterSpinsInTimeout = 0;
unsigned long windCycleDuration = 0;
unsigned long windTimeout = 0;
unsigned long windLastGust = 0;
float windActualSpeed = 0;
float windKMH(float windMS);
float windLongPeriodSpeed = 0;
float gust = 0;

bool meteoSwitch = true; // = USE_METEO
bool aprsSwitch = true;  // = Use_IGATE
bool digiSwitch = true;  // = USE_DIGIPEATER


// -------------------------------------------------------------------------------------------
// ---cgw----------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------


//char Build[7] = "230729";

// --- gli info sulle stringhe georeferenziate fisso per iGate - variabile per la meteo --- //
//char igate_info[52]="";
char meteo_info[52]="";   // variabile da accodare alla stringa meteo georeferenziata
//char igate_info[53] = "https://github.com/iw1cgw/LoRa_APRS_Meteo_dds-cgw";

// --- gli status sulle stringhe senza georeferenziazione --- //
// per iGate viene indicato il valore dell'ultimo aascolto
// per meteo viene indicato il bannet di GitHub

void menu();
void righello();
void banner();
void status_display();
void initial_reset();
void load_param();
//void latlon_meteo_eraser();
//void latlon_igate_eraser();

void EEPROM_eraser(byte start, byte stop);
void EEPROM_writer(byte start, byte stop, char tmp_data[50]);
void EEPROM_loader(byte start, byte stop, char tmp_data[50]);
void array_eraser(byte start, byte stop, char tmp_data[50]);
void verifica_parametri();
void APRS_LatLon();

char lat_meteo[10] ="";
char lon_meteo[11] ="";

char lat_meteo_APRS[10];
char lon_meteo_APRS[11];

char lat_igate[10] = "";
char lon_igate[11] = "";

char lat_igate_APRS[10];
char lon_igate_APRS[11];

char frequencyC[7]="433775";

char WiFi_ssiD[21]="";
char WiFi_pwd[21]= "";

char altitude[5]="";

char call[7];
String IGATE_CALLSIGN;
String METEO_CALLSIGN;

byte meteo_ssiD=3;
byte igate_ssiD=10;

char aprs_passcode[6] = "";
char aprs_server[21] = "";

unsigned LoRa_power=2;
byte cnt_meteo_send=0;
byte tx_interval=10;

float drift_therm=0;
char drift_thermC[6];

  //Adafruit_BME280 bme; // use I2C interface
  Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
  Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
  Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();;

sensors_event_t humidity_event, temp_event, pressure_event;


byte ch_term=0;

byte ptr;
byte sep;

unsigned tmp;

float minHum;
float maxHum;

char tmp_buffer[60]="";            // a supporto seriale e menu e caricamento EEPROM
char carMenu;                   // a supporto seriale e menu
char car;                       // a supporto seriale e menu


byte cnt=0;                   // contatore invio stringhe meteo


#define  EEPROM_SIZE  254  // // EEPROM size puo' indirizzare da 0 a 255
#define bottom   ".. (m)enu - (d)isplay - (s)end meteo - (r)ead sensor ..\n"



// -------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------




void setup() {
  
 //Init EEPROM
  
  Serial.begin(SERIAL_BAUD);
  Serial.println( "\n" + String(Project) + " v." + String(Release) + "\nmodified by IW1CGW based on OK2DDS' project\n");
   
  EEPROM.begin(EEPROM_SIZE);
  load_param();
  verifica_parametri();
  lora_setup();
  delay(25);


  BM_sensor_status = false;
  BMPstatus = false;
  BMEstatus = false;
  AHTstatus = false;

  //------------------------------ test del AHT20
  if (!aht.begin()) {}
  else
    {
      Serial.println("AHT20 OK");
      AHTstatus = true;
    } 
  
  //------------------------------ test del BME280
  if (AHTstatus == false)
    {
      if (!bme.begin(0x76)) {}
      else 
        {
          BMEstatus = true;
          BM_sensor_status = true;
          Serial.println("BME280 0x76 OK");
        }
    }

//------------------------------ test del BMP280 su 2 possibili indirizzi 0x77 se integrato con AHT20 o 0x76 se stand alone

  if (AHTstatus == true && BMEstatus == false)
    {
      if (!bmp.begin(0x77)) {}
      else
        {
          Serial.println("BMP280 0x77 OK");
          BMPstatus = true;
          BM_sensor_status = true;
        }
    }
 
 if (AHTstatus == false && BMEstatus == false)
    {
      if (!bmp.begin(0x76)) {}
      else
        {
          Serial.println("BMP280 0x76 OK");
          BMPstatus = true;
          BM_sensor_status = true;
        }
    }


  if (BM_sensor_status == false) Serial.println(F("no sensor BMP/BME found"));
  
  // HALL - digital
  if (USE_ANEMOMETER) pinMode(HALL_SENSOR_PIN, INPUT);
  //attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), hall_change, HIGH);

  voltage = float(analogRead(HALL_SENSOR_PIN)) / 4095*2*3.3*1.1;
  delay(10);

  if (Use_WiFi) {
    WiFi.setAutoReconnect(true);
    WiFi.setHostname(Hostname);
    WiFi.begin(WiFi_ssiD, WiFi_pwd);

    Serial.println("Connecting to WiFi..");
    delay(500);
    wifiStatus = true;
    myIP = ipToString(WiFi.localIP());
    Serial.println("IP: " + myIP);
    server.begin();
    ws.onEvent(onWsEvent);
    serverWS.addHandler(&ws);
    serverWS.begin();
    delay(500);
    if (aprsSwitch && check_wifi()) {
      aprsis_connect();
    }
    delay(500);
  }

  Serial.println("Startup finished.\n");
  status_display();
  delay(500);
  if (meteoSwitch) lastMtBeacon = millis() - int(tx_interval * 60000);
  delay(500);
  if (aprsSwitch) lastIgBeacon = millis() - int(tx_interval * 60000);
  
}

void loop() {

/*
  ---------------------------------------------------------------------------
    LETTURA FLUSSI SULLA SERIALE

    i dati utili si intendono quelli racchiusi entro la trama che
    inizia con '[' e termina con ']'
    i valori racchiusi sono incapsulati nella String 'tmp_string'
  ---------------------------------------------------------------------------
  */
    car = Serial.read();
    if (car == 'm' )
      {
        while (Serial.read() != '\n') {};
        menu();
      }

    if (car == 'd' )
      {
        while (Serial.read() != '\n') {};
        status_display();
      }

    if (car == 't' )
      {
        while (Serial.read() != '\n') {};
        beacon_igate();
      }

    if (car == 's' )
      {
        while (Serial.read() != '\n') {};
        beacon_meteo();
      }


    if (car == 'r' )
      {
        while (Serial.read() != '\n') {};
        
        righello();
        
        if ( BM_sensor_status )
          {
            float temp = getTempC();
            String stemp = String(temp,1);
            float press = getPressure();
            String spress = String(press,1);
            float Hum = getHum();
            String sHum = String(Hum);

            Serial.print(F("therm: "));
            Serial.print(stemp);
            Serial.println(F(" C."));
                
            Serial.print(F("hum: "));
            Serial.print(sHum);
            Serial.println(F(" %"));

            Serial.print(F("press SLM: "));
            Serial.print(spress);
            Serial.println(F(" hPA"));

            righello();
        
          }
      }


    if (car == '#')          
      {
        while (Serial.read() != '\n') {};
        EEPROM_eraser(0,250);
        Serial.println(F("EEPROM erased"));
      }

    if (car == 'e')          
      {
        while (Serial.read() != '\n') {};
        tmp = 0;
        while (tmp <= 250)
          {
           Serial.print(tmp); Serial.print(F(" - ")); Serial.println(EEPROM.read(tmp));
           tmp++;    
          } 
      }

    
  if (check_wifi() && wifiStatus == false) {
    wifiStatus = true;
    Serial.println("Wi-Fi connected");
  }
  if (Use_WiFi && !check_wifi() && wifiStatus) {
    wifiStatus = false;
    Serial.println("Wi-Fi not connected!");
    WiFi.reconnect();
  }
  if (Use_WiFi && check_wifi()) {
    if (aprsSwitch && aprsSwitch && !check_aprsis() && aprsLastReconnect + 60000 < millis()) {
      aprsis.stop();
      delay(100);
      aprsLastReconnect = millis();
      aprsis_connect();
    }
    if (myIP != ipToString(WiFi.localIP())) {
      myIP = ipToString(WiFi.localIP());
      Serial.println("IP: " + myIP);
    }
    WiFiClient client = server.available();
    if (client) {                             
    unsigned long currentTime = millis();
    unsigned long previousTime = currentTime;
    String header;
    Serial.println("HTTP from " + ipToString(client.remoteIP()) + ":" + String(client.remotePort()));     
    String currentLine = "";         
    while (client.connected() && currentTime - previousTime <= 5000) {
      currentTime = millis();
      if (client.available()) { 
        char c = client.read();
        header += c;
        if (c == '\n') {
          if (currentLine.length() == 0) {
            if (HTTP_DEBUG_MODE) Serial.println(header);
            client.println("HTTP/1.1 200 OK");
            if (GETIndex(header, "/api/meteo"))
              client.println("Content-type:text/csv");
            else if (header.indexOf("json") >= 0)
              client.println("Content-type:application/json");
            else
              client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();
            if (header.indexOf("favicon.ico") == -1) {
            if (GETIndex(header, "/api")) {
              // API responses without user frontend layout
              if (GETIndex(header, "/api/meteo"))
                client.println(tempToWeb(getTempC()) + "," + "," + HumToWeb(int(getHum())) + pressToWeb(getPressure()) );
              if (GETIndex(header, "/api/graphs-json"))
                client.println("{\"temperature\": [" + String(tempValues) + "], \"pressure\": [" + String(pressValues) + "], \"Hum\": [" + String(HumValues) + "], \"wind\": [" + String(windValues) + "]}");
              if (GETIndex(header, "/api/json"))
                //client.println("{\"general\": {\"version\":\"" + String(Release) + "\", \"destcall\":\"" + String(DESTCALL) + "\", \"system_time\":" + String(millis()) + ", \"voltage\":" + String(voltage) + ", \"battery\":" + String(battPercent) + ", \"wifi_status\":" + (check_wifi() ? "true" : "false") + ", \"wifi_signal_db\":" + (check_wifi() ? String(WiFi.RSSI()) : "0") + ", \"wifi_ssid\":\"" + String(WiFi.SSID()) + "\", \"wifi_hostname\":\"" + String(Hostname) + "\", \"bmp280_status\":" + (getBM_sensor_status() ? "true" : "false") + "}, \"lora\": {\"METEO_CALLSIGN\":\"" + String(METEO_CALLSIGN) + "\", \"meteo_enabled\":" + (meteoSwitch ? "true" : "false") + ", \"igate_callsign\":\"" + String(IGATE_CALLSIGN) + "\", \"aprs_is_enabled\":" + (aprsSwitch ? "true" : "false") + ", \"aprs_is_status\":" + (check_aprsis() ? "true" : "false") + ", \"aprs_server\":\"" + (check_aprsis() ? String(APRSISServer) : "disconnected") + "\", \"hall_sensor\":" + String(anemoACValue) + ", \"last_rx\":\"" + String(lastRXstation) + "\"" + "}, \"meteo\": {\"temperature\":" + valueForJSON(tempToWeb(getTempC())) + ", \"pressure\":" + valueForJSON(pressToWeb(getPressure()))  + ", \"Hum\":" + valueForJSON(HumToWeb(getHum())) + ", \"actual_wind\":" + valueForJSON(windToWeb(windActualSpeed)) + ", \"long_period_wind\":" + valueForJSON(windToWeb(windLongPeriodSpeed)) + ", \"gust\":" + valueForJSON(windToWeb(gust)) + ", \"min_temperature\":" + (getBM_sensor_status() ? String(minTemp) : "0") + ", \"max_temperature\":" + (getBM_sensor_status() ? String(maxTemp) : "0") + ", \"min_pressure\":" + (getBM_sensor_status() ? String(minPress) : "0") + ", \"max_pressure\":" + (getBM_sensor_status() ? String(maxPress) : "0") + ", \"max_wind\":" + String(maxWind) + ", \"max_gust\":" + String(maxGust) + "}}");
                client.println("{\"general\": {\"version\":\"" + String(Release) + "\", \"destcall\":\"" + String(DESTCALL) + "\", \"system_time\":" + String(millis()) + ", \"voltage\":" + String(voltage) + ", \"battery\":" + String(battPercent) + ", \"wifi_status\":" + (check_wifi() ? "true" : "false") + ", \"wifi_signal_db\":" + (check_wifi() ? String(WiFi.RSSI()) : "0") + ", \"wifi_ssid\":\"" + String(WiFi.SSID()) + "\", \"wifi_hostname\":\"" + String(Hostname) + "\", \"bmp280_status\":" + (getBM_sensor_status() ? "true" : "false") + "}, \"lora\": {\"METEO_CALLSIGN\":\"" + String(METEO_CALLSIGN) + "\", \"digi_enabled\":" + (digiSwitch ? "true" : "false") + "\", \"meteo_enabled\":" + (meteoSwitch ? "true" : "false") + ", \"igate_callsign\":\"" + String(IGATE_CALLSIGN) + "\", \"aprs_is_enabled\":" + (aprsSwitch ? "true" : "false") + ", \"aprs_is_status\":" + (check_aprsis() ? "true" : "false") + ", \"aprs_server\":\"" + (check_aprsis() ? String(APRSISServer) : "disconnected") + "\", \"hall_sensor\":" + String(anemoACValue) + ", \"last_rx\":\"" + String(lastRXstation) + "\"" + "}, \"meteo\": {\"temperature\":" + valueForJSON(tempToWeb(getTempC())) + ", \"pressure\":" + valueForJSON(pressToWeb(getPressure()))  + ", \"Hum\":" + valueForJSON(HumToWeb(getHum())) + ", \"actual_wind\":" + valueForJSON(windToWeb(windActualSpeed)) + ", \"long_period_wind\":" + valueForJSON(windToWeb(windLongPeriodSpeed)) + ", \"gust\":" + valueForJSON(windToWeb(gust)) + ", \"min_temperature\":" + (getBM_sensor_status() ? String(minTemp) : "0") + ", \"max_temperature\":" + (getBM_sensor_status() ? String(maxTemp) : "0") + ", \"min_pressure\":" + (getBM_sensor_status() ? String(minPress) : "0") + ", \"max_pressure\":" + (getBM_sensor_status() ? String(maxPress) : "0") + ", \"max_wind\":" + String(maxWind) + ", \"max_gust\":" + String(maxGust) + "}}");
            } else {
            client.println(String(webPageStart));
            if (!GETIndex(header, "/watch")) client.println(String(webPageHeader) + "</h1><br>");
            if (GETIndex(header, "/switch-meteo")) {
              meteoSwitch = !meteoSwitch;
              if (meteoSwitch == true ) EEPROM.write(165,1);
              if (meteoSwitch == false) EEPROM.write(165,0);
              EEPROM.commit();
              client.println(webReload);
            }
            if (GETIndex(header, "/switch-aprs")) {
              aprsSwitch = !aprsSwitch;
              if (aprsSwitch == true ) EEPROM.write(166,1);
              if (aprsSwitch == false) EEPROM.write(166,0);
              EEPROM.commit();
              client.println(webReload);
              aprsis.stop();
              delay(100);
            }
            if (GETIndex(header, "/switch-digi")) {
              digiSwitch = !digiSwitch;
              if (digiSwitch == true ) EEPROM.write(167,1);
              if (digiSwitch == false) EEPROM.write(167,0);
              EEPROM.commit();
              client.println(webReload);
              delay(100);
            }

            if (GETIndex(header, "/restart")) {
              client.println();
              ESP.restart();
            }
            if (GETIndex(header, "/reset-tx")) {
              lastUpload = millis();
              lastIgBeacon = millis();
              lastMtBeacon = millis();
              client.println("<br>TX reset done.<br>");
            }
            if (GETIndex(header, "/reset-temp")) {
              minTemp = getTempC();
              maxTemp = minTemp;
              client.println("<br>Temperature reset done.<br>");
            }
            if (GETIndex(header, "/reset-hum")) {
              minHum = getHum();
              maxHum = minHum;
              client.println("<br>Humidity reset done.<br>");
            }

            if (GETIndex(header, "/reset-press")) {
              minPress = getPressure();
              maxPress = minPress;
              client.println("<br>Pressure reset done.<br>");
            }
            if (GETIndex(header, "/reset-bmp")) {
              minTemp = getTempC();
              maxTemp = minTemp;
              minPress = getPressure();
              maxPress = minPress;
              client.println("<br>BMP values reset done.<br>");
            }
            if (GETIndex(header, "/reset-wind")) {
              maxWind = windLongPeriodSpeed;
              maxGust = windActualSpeed;
              client.println("<br>Wind reset done.<br>");
            }
            if (GETIndex(header, "/lora"))
              // --- original --- // client.println("<br>Version: " + String(VERSION) + "<br><br> Voltage: " + String(voltage) + "V<br>Battery: " + String(battPercent) + "%<br>Wi-Fi: " + (check_wifi() ? String(WiFi.SSID()) + " " + String(WiFi.RSSI()) + " dB<br>IP: " + ipToString(WiFi.localIP()) : String("not connected")) + String("<br>APRS-IS: ") + (aprsis.connected() ? "connected" : "not connected") + "<br>Last RX: " + String(lastRXstation) + "<br>Hall sensor: " + String(anemoACValue) + "<br><br>");
              client.println("Version: " + String(Release) + "<br><br> Voltage: " + String(voltage) + " - Battery: " + String(battPercent) + "%<br>Wi-Fi: " + (check_wifi() ? String(WiFi.SSID()) + " " + String(WiFi.RSSI()) + " dB<br>IP: " + ipToString(WiFi.localIP()) : String("not connected")) + String("<br>APRS-IS: ") + (aprsis.connected() ? "connected" : "not connected") + "<br>Last RX: " + String(lastRXstation) + "<br><br>");
            if ((GETIndex(header, "/lora") || GETIndex(header, "/min-max")) && meteoSwitch && (BM_sensor_status || USE_ANEMOMETER)) {
              client.println("<table><tr><td></td><td><b>Minimum</b></td><td><b>Maximum</b></td></tr>");
              
              if (BM_sensor_status) {
                client.println("<tr><td><b>Temperature</b></td><td>" + String(minTemp) + " &deg;C</td><td>" + String(maxTemp) + " &deg;C</td></tr>");
                client.println("<tr><td><b>PressureSLM</b></td><td>" + String(minPress) + " hPa</td><td>" + String(maxPress) + " hPa</td></tr>");
              }
              
              if (AHTstatus == true || BMEstatus == true ) {  
                client.println("<tr><td><b>% Hum</b></td><td>" + String(minHum) + " %</td><td>" + String(maxHum) + " %</td></tr>");
              }
              
              if (USE_ANEMOMETER) {
                client.println("<tr><td><b>Avg. wind</b></td><td>0.00 m/s</td><td>" + String(maxWind) + " m/s</td></tr>");
                client.println("<tr><td><b>Wind gust</b></td><td>0.00 m/s</td><td>" + String(maxGust) + " m/s</td></tr>");
              }
              client.println("</table><br>");
            }
            if (GETIndex(header, "/lora")) {
              // --- original --- // client.println("<br>Reset values<br><a href='/reset-bmp'>BMP values</a> - <a href='/reset-temp'>Temperature</a> - <a href='/reset-press'>Pressure</a> - <a href='/reset-wind'>Wind</a><br>");
              
              if (AHTstatus == true || BMEstatus == true )   client.println("<br>Reset values<br><a href='/reset-temp'>Temperature</a> - <a href='/reset-hum'>Humidity</a> - <a href='/reset-press'>Pressure</a><br>");
              else client.println("<br>Reset values<br><a href='/reset-temp'>Temperature</a> - <a href='/reset-press'>Pressure</a><br>");
       
              client.println("<br><a href='/switch-meteo'>Turn meteo On/Off</a> (" + String(meteoSwitch ? "ON" : "OFF") + ")");
              client.println("<br><a href='/switch-aprs'>Turn IGate On/Off</a> (" + String(aprsSwitch ? "ON" : "OFF") + ")");
              client.println("<br><a href='/switch-digi'>Turn digipeater On/Off</a> (" + String(digiSwitch ? "ON" : "OFF") + ")");
              // --- original --- // client.println("<br><a href='/change-aprsis'>Change APRS-IS server</a> (" + String(APRSISServer) + ")");
              client.println("<br><a href='/restart'>Restart device</a>");
              client.println("<br><br><a href='/'>View main meteo page</a>");
            }
            if (GETIndex(header, "/graphs")) {
              if (BM_sensor_status || USE_ANEMOMETER)
                client.println("<script src='https://cdnjs.cloudflare.com/ajax/libs/Chart.js/2.9.4/Chart.js'></script>");
              else
                client.println("<br>No graphs to display.<br>");
              if (BM_sensor_status) {
                client.println(generateGraph(tempValues, "Temperature", "temp", 230, 0, 0));
                client.println(generateGraph(pressValues, "Pressure", "press", 0, 125, 0));
              }  

              if (AHTstatus == true || BMEstatus == true ) {
                client.println(generateGraph(HumValues, "% Hum", "Hum", 0, 100, 0));
              }

              if (USE_ANEMOMETER)
                client.println(generateGraph(windValues, "Average wind (m/s)", "wind", 0, 0, 255));
              client.println("<a href='/'>View main meteo page</a>");
            }
            if (GETIndex(header, "/tx")) {
              pktIndex = header.indexOf("/tx/");
              client.println("OK");
              Serial.println("Custom packet via TCP");
              String custPkt;
              custPkt = header.substring(pktIndex + 4, header.indexOf("HTTP/") - 1);
              while (custPkt.indexOf("%3E") != -1) {
                custPkt = custPkt.substring(0, custPkt.indexOf("%3E")) + ">" + custPkt.substring(custPkt.indexOf("%3E") + 3);
              }
              while (custPkt.indexOf("%20") != -1) {
                custPkt = custPkt.substring(0, custPkt.indexOf("%20")) + " " + custPkt.substring(custPkt.indexOf("%20") + 3);
              }
              while (custPkt.indexOf("%3A") != -1) {
                custPkt = custPkt.substring(0, custPkt.indexOf("%3A")) + ":" + custPkt.substring(custPkt.indexOf("%3A") + 3);
              }
              while (custPkt.indexOf("%2F") != -1) {
                custPkt = custPkt.substring(0, custPkt.indexOf("%2F")) + "/" + custPkt.substring(custPkt.indexOf("%2F") + 3);
              }
              while (custPkt.indexOf("%40") != -1) {
                custPkt = custPkt.substring(0, custPkt.indexOf("%40")) + "@" + custPkt.substring(custPkt.indexOf("%40") + 3);
              }
              while (custPkt.indexOf("%7B") != -1) {
                custPkt = custPkt.substring(0, custPkt.indexOf("%7B")) + "{" + custPkt.substring(custPkt.indexOf("%7B") + 3);
              }
              lora_send(custPkt);
              custPkt = "";
              pktIndex = 0;
              delay(3000);
            }
            if (GETIndex(header, "/change-aprsis")) {
              client.println(webAPRSISChangePrompt);
            }
            if (GETIndex(header, "/new-aprsis")) {
              apktIndex = header.indexOf("GET /new-aprsis");
              String newServer = header.substring(apktIndex + 16, header.indexOf("HTTP/") - 1);
              Serial.println("New APRS-IS server request: " + String(newServer));
              if (newServer != "null" && Use_WiFi && aprsSwitch && check_wifi()) {
                String backupServer = APRSISServer;
                APRSISServer = newServer;
                aprsis.stop();
                delay(100);
                aprsis_connect();
                delay(100);
                if (check_aprsis() && check_wifi()) {
                  client.println(webAPRSISChangeSuccess);
                  Serial.println("Success. New server: " + String(APRSISServer));
                } else {
                  client.println(webAPRSISChangeError);
                  Serial.println("Connection not successful.");
                  APRSISServer = backupServer;
                  aprsis.stop();
                  delay(100);
                  if(aprsSwitch) aprsis_connect();
                }
              } else {
                client.println(webAPRSISChangeError);
                Serial.println("Bad input. Change not successful.");
              }
              apktIndex = 0;
            }
            if (GETIndex(header, "/ ")) {
              // ORDINARY METEO WEBSITE
              client.println(webMeteoOnlineIndicator);
              client.println(webMeteoLayout);
              client.println(webSocketSetupScript);
              client.println(HTMLelementDef("onlineIndicator") + HTMLelementDef("temp") + HTMLelementDef("Hum") + HTMLelementDef("press") + HTMLelementDef("wind") + HTMLelementDef("windkmh") + HTMLelementDef("gust") + HTMLelementDef("windlp"));
              
              client.println(webMeteoOnlineRoutine);
              client.println(webSocketHandleScript);
            }
            if (GETIndex(header, "/watch")) {
              // METEO WEBSITE LAYOUT FOR WATCH
              client.println(webMeteoWatchLayout);
              client.println("<script>var dat = '" + tempToWeb(getTempC()) + "," + HumToWeb(int(getHum())) + pressToWeb(getPressure()) + "," + windToWeb(windActualSpeed) + "," + windToWeb(windKMH(windActualSpeed)) + "'; ");
              client.println(HTMLelementDef("temp") + HTMLelementDef("Hum") +  HTMLelementDef("press") + HTMLelementDef("wind") + HTMLelementDef("windkmh"));
                          
              client.println(webWatchValuesScript);
            }
            if (!GETIndex(header, "/watch"))
              client.println(webPageFooter);
            client.println(webPageEnd);
            }
            }
            client.println();
            break;
          } else {
            currentLine = "";
          }
        } else if (c != '\r') {
          currentLine += c;
        }
      }
    }
    client.stop();
    }
  }

  ws.cleanupClients();
  if (Use_WiFi && isWSconnected && lastWSupdate + 700 < millis()) updateWebSocket();

  if (aprsSwitch && check_wifi() && check_aprsis() && lastIgBeacon + (tx_interval * 60000) < millis()) beacon_igate();
  if (meteoSwitch && lastMtBeacon + (tx_interval * 60000) < millis()) beacon_meteo();
  if (Use_UPLOAD && check_wifi() && lastUpload + (UPLOAD_TIMEOUT * 60000) < millis()) beacon_upload();

  voltage = float(analogRead(HALL_SENSOR_PIN)) / 4095*2*3.3*1.1;
  battPercent = 100 * (voltage - 3.3) / (4.2 - 3.3);
  if (battPercent > 100) battPercent = 100;
  if (battPercent < 0) battPercent = 0;

  int packetSize = LoRa.parsePacket();
  if (packetSize) {
  while (LoRa.available()) {
    bool digiOutput = false;
    String destCall, digiPath, originalPath, sourceCall, message, digiPacket, statusMessage;
    int pos1, pos2;
    String rxPacket = LoRa.readString();
    
    // --- original rxPacket = rxPacket.substring(3);
    rxPacket = rxPacket.substring(3,(rxPacket.length()-1));
    Serial.println("RX: " + rxPacket);

    if (!(rxPacket.length() < 5 || rxPacket.indexOf('>') < 5 || rxPacket.indexOf(':') < rxPacket.indexOf('>') || rxPacket.substring(rxPacket.indexOf('>') + 1, rxPacket.indexOf(':')) == "") && aprsSwitch && Use_WiFi && aprsSwitch) {
      String igatePacket = rxPacket;
      if (igatePacket.indexOf("NOGATE") == -1 && igatePacket.indexOf("RFONLY") == -1 && igatePacket.indexOf("TCPIP") == -1 && igatePacket.indexOf("TCPXX") == -1 && igatePacket.indexOf(String(METEO_CALLSIGN)) == -1 && igatePacket.indexOf(String(IGATE_CALLSIGN) + "*") == -1 && rxPacket.substring(0, rxPacket.indexOf('>')) != String(IGATE_CALLSIGN)) {
        igatePacket = igatePacket.substring(0, igatePacket.indexOf(":")) + ",qAO," + String(IGATE_CALLSIGN) + igatePacket.substring(igatePacket.indexOf(":"));
        //Serial.println("IGated packet.");
        aprsis_send(igatePacket);
      }
    }

      // digipeating
      pos1 = rxPacket.indexOf('>');
      if (pos1 < 3)
        goto bad_packet;
      sourceCall = rxPacket.substring(0, pos1);
      pos2 = rxPacket.indexOf(':');
      if (pos2 < pos1)
        goto bad_packet;
      destCall = rxPacket.substring(pos1 + 1, pos2);
      message = rxPacket.substring(pos2 + 1);
      digiPath = "";
      pos2 = destCall.indexOf(',');
      if (pos2 > 0) {
        digiPath = destCall.substring(pos2 + 1);
        destCall = destCall.substring(0, pos2);
      }
      originalPath = digiPath;
      if (destCall == "")
        goto bad_packet;
      lastRXstation = sourceCall;
      if (int callIndex = digiPath.indexOf(String(IGATE_CALLSIGN)) > -1 && digiPath.indexOf(String(IGATE_CALLSIGN) + "*") == -1) {
        digiPath = digiPath.substring(0, callIndex - 1) + digiPath.substring(callIndex + String(IGATE_CALLSIGN).length());
      }
      if (int paradigmIndex = digiPath.indexOf("WIDE1-") > -1 && digiSwitch && digiPath.indexOf(String(IGATE_CALLSIGN) + "*") == -1 && rxPacket.indexOf(String(METEO_CALLSIGN)) == -1 && sourceCall.indexOf(String(IGATE_CALLSIGN)) == -1) {
        paradigmIndex = digiPath.indexOf("WIDE1-");
        if (paradigmIndex == 0)
          paradigmIndex = 1;
        //Serial.println(digiPath.substring(0, paradigmIndex - 1));
        if (paradigmIndex == 1)
          digiPath = digiPath.substring(0, paradigmIndex - 1) + "," + String(IGATE_CALLSIGN) + "*,WIDE1*" + digiPath.substring(paradigmIndex + 6);
        else
          digiPath = digiPath.substring(0, paradigmIndex - 1) + "," + String(IGATE_CALLSIGN) + "*,WIDE1*" + digiPath.substring(paradigmIndex + 7);
        // add SNR and RSSI
        //message = message + " SNR=" + String(LoRa.packetSnr()) + "dB RSSI=" + String(LoRa.packetRssi()) + "dB";
        if (digiPath.indexOf(",") != 0)
          digiPath = "," + digiPath;
        digiPacket = sourceCall + ">" + destCall + digiPath + ":" + message;
        lora_send(digiPacket);
      } else if (digiSwitch && DIGI_IGNORE_PARADIGM && digiPath.indexOf("*") == -1 && (millis() > lastDigipeat + 600000 || lastDigipeat == 0) && digiPath.indexOf(String(IGATE_CALLSIGN) + "*") == -1 && rxPacket.indexOf(String(METEO_CALLSIGN)) == -1 && sourceCall.indexOf(String(IGATE_CALLSIGN)) == -1) {
        lastDigipeat = millis();
        digiPath = digiPath + "," + String(IGATE_CALLSIGN) + "*";
        if (digiPath.indexOf(",") != 0)
          digiPath = "," + digiPath;
        // do not add SNR and RSSI
        digiPacket = sourceCall + ">" + destCall + digiPath + ":" + message;
        lora_send(digiPacket);
	    // do not digipeat without WIDE1-1
      } else if (digiSwitch && DIGI_IGNORE_PARADIGM) {
        Serial.println("Station not repeated.");
      }
      digiOutput = true;

      // send status
      statusMessage = String(IGATE_CALLSIGN) + ">" + String(DESTCALL) + ":>Last RX: " + String(sourceCall) + " SNR=" + String(LoRa.packetSnr()) + "dB RSSI=" + String(LoRa.packetRssi()) + "dB";
      if (aprsSwitch && USE_LASTRX_STATUS && originalPath.indexOf("*") == -1)
        aprsis_send(statusMessage);

    bad_packet:
      if (!digiOutput) Serial.println("Bad packet");
  }
  }

  while (aprsSwitch && check_aprsis() && aprsis.available()) {
    String apstring;
    char aprx = aprsis.read();
    apstring += aprx;
    if (aprx == '\n') {
      //Serial.println(apstring);
      if (apstring.indexOf("logresp") == -1) {
        // incoming packet handling
      }
      apstring = "";
    }
  }

  if (USE_ANEMOMETER) {
  anemoACValue = analogRead(HALL_SENSOR_PIN);
  if (ANEMO_DEBUG_MODE) Serial.println(anemoACValue);
  if (anemoACValue <= ANEMO_AC_THRESHOLD && !magnetDetected) {
    magnetDetected = true;
    hall_change();
  } else if (anemoACValue >= ANEMO_AC_LOSE) {
    magnetDetected = false;
  }
  if (windMeterSpins >= int(ANEMO_RECALC_LIMIT)) {
    windActualSpeed = float(1 / ((millis() - windCycleDuration) / (windMeterSpins * ANEMOMETER_LENGTH * 1000)));
    //windActualSpeed = float(windMeterSpins * 1000 / (millis() - windCycleDuration)) * float(ANEMOMETER_LENGTH);
    windCycleDuration = millis();
    Serial.println("Wind: " + String(windActualSpeed) + " m/s");
    windMeterSpins = 0;
    if (windActualSpeed > gust) {
      gust = windActualSpeed;
      windLastGust = millis();
    }
  }
  if (windTimeout + (ANEMO_RECALC_LIMIT_TIMEOUT * 1000) < millis()) {
    windLongPeriodSpeed = float(1 / ((millis() - windTimeout) / (windMeterSpinsInTimeout * ANEMOMETER_LENGTH * 1000)));
    //windLongPeriodSpeed = float(windMeterSpinsInTimeout * 1000 / (millis() - windTimeout)) * float(ANEMOMETER_LENGTH);;
    windTimeout = millis();
    Serial.println("Long-period wind: " + String(windLongPeriodSpeed) + " m/s");
    windMeterSpinsInTimeout = 0;
  }
  if (millis() > windCycleDuration + (ANEMO_RECALC_ACTUAL_SPEED * 1000)) windActualSpeed = 0;
  if (millis() > windLastGust + (ANEMO_RECALC_LIMIT_TIMEOUT * 1000)) gust = windActualSpeed;
  }
}

void lora_setup() {
  SPI.begin(LoRa_SCK, LoRa_MISO, LoRa_MOSI, LoRa_SS);
  LoRa.setPins(LoRa_SS, LoRa_RST, LoRa_DIO0);
  if (!LoRa.begin(atoi(frequencyC)*1000)) {
    Serial.println("Failed to setup LoRa module.");
    while (1);
  }
  LoRa.setSpreadingFactor(LoRa_SpreadingFactor);
  LoRa.setSignalBandwidth(LoRa_SignalBandwidth);
  LoRa.setCodingRate4(LoRa_CodingRate4);
  LoRa.enableCrc();
  LoRa.setTxPower(LoRa_power);
  delay(3000);
  if (!aprsSwitch) {
    LoRa.sleep();
  }
}

void lora_send(String tx_data) {
  LoRa.setFrequency(atoi(frequencyC)*1000);
  LoRa.beginPacket();
  LoRa.write('<');
  LoRa.write(0xFF);
  LoRa.write(0x01);
  Serial.println("TX: " + tx_data);
  LoRa.write((const uint8_t *)tx_data.c_str(), tx_data.length());
  LoRa.endPacket();
  LoRa.setFrequency(atoi(frequencyC)*1000);
  if (!aprsSwitch) {
    LoRa.sleep();
  }
}

void aprsis_connect() {
  if (Use_WiFi && aprsSwitch) {
    if (aprsis.connect(APRSISServer.c_str(), APRS_IS_Port)) {
      Serial.println("APRS-IS OK");
    } else {
      Serial.println("APRS-IS error");
    }
    if (check_wifi() && check_aprsis()) {
      aprsis.println("user " + String(IGATE_CALLSIGN) + " pass " + String(aprs_passcode) + " vers LoRa_APRS_Meteo " + String(Release));
      aprsSwitch = true;
    }
  }
}

void upload_data(String upload_data) {
  Serial.println("HTTP: " + String(upload_data));
  if (check_wifi()) {
  String path = String(SERVER_URL) + upload_data;
  upload.begin(path.c_str());
  int response = upload.GET();
  if (response > 0) {
    Serial.println("HTTP: " + String(response));
  } else {
    Serial.println("HTTP ERROR: " + String(response));
  }
  }
}

void aprsis_send(String aprsis_packet) {
  if (!check_aprsis() && aprsSwitch && aprsSwitch && check_wifi())
   {
    aprsis.stop();
    delay(100);
    aprsis_connect();
  } else if (check_wifi() && check_aprsis()) {
    Serial.println("APRS-IS: " + String(aprsis_packet));
    aprsis.println(aprsis_packet);
  } else {
    Serial.println("APRS-IS TX error");
  }
}

void beacon_igate() {
  lastIgBeacon = millis();
  if (aprsSwitch && check_wifi()) {
    String beacon = String(IGATE_CALLSIGN) + ">" + String(DESTCALL) + ":!" + String(lat_igate_APRS) + "L" + String(lon_igate_APRS) + "&" + String(igate_info) + String(" | batt:") + String(voltage)+"V";
    if (IGATE_BCN_NETWORK) {
      aprsis_send(beacon);
    } else if (aprsSwitch) {
      lora_send(beacon);
    }
    //if (check_wifi() && USE_METEO_STATUS) beacon_meteo_status();
  }
}

void beacon_meteo() {
  
  lastMtBeacon = millis();

  if  (cnt_meteo_send > METEO_STATUS_SEND_INTERVAL ) cnt_meteo_send=0;
  beacon_meteo_status();
  cnt_meteo_send++;


  if (BM_sensor_status) {
    float temp = getTempC();
    float press = getPressure();
    String stemp = String(temp,1);
    String spress = String(press,1);
    tempValues = addGraphValue(tempValues, stemp);
    if (temp < minTemp || minTemp == -1000) minTemp = temp;
    if (temp > maxTemp) maxTemp = temp;
    pressValues = addGraphValue(pressValues, spress);
    if (press < minPress || minPress == -1000) minPress = press;
    if (press > maxPress) maxPress = press;
  } else {
    tempValues = addGraphValue(tempValues, "N/A");
    pressValues = addGraphValue(pressValues, "N/A");
  }

  if ( AHTstatus == true || BMEstatus == true )
   {
    float Hum = getHum();
    String sHum = String(Hum);
    HumValues = addGraphValue(HumValues, sHum);
    if ( Hum < minHum || minHum == 0 ) minHum = Hum;
    if ( Hum > maxHum ) maxHum = Hum;
    }
  else {
    HumValues = addGraphValue(HumValues, "N/A");
  }
  
  if (meteoSwitch && BM_sensor_status) {
     String meteoBeacon = String(METEO_CALLSIGN) + ">" + String(DESTCALL) + ":!" + String(lat_meteo_APRS) + "/" + String(lon_meteo_APRS) + "_.../" + String(windSpeedAPRS(windLongPeriodSpeed)) + "g" +  String(windSpeedAPRS(gust)) + "t" + String(getTempAPRS()) + "r...p...P..." + "h" + String(getHumAPRS()) + "b" + String(getPressureAPRS())+"." + String(meteo_info);
     //Serial.println(meteoBeacon);
     //lora_send(meteoBeacon);
     aprsis_send(meteoBeacon);
    

}


  if (USE_ANEMOMETER) {
    windValues = addGraphValue(windValues, String(windLongPeriodSpeed));
    if (windLongPeriodSpeed > maxWind) maxWind = windLongPeriodSpeed;
    if (gust > maxGust) maxGust = gust;
  } else {
    windValues = addGraphValue(windValues, "N/A");
  }
  gust = 0;
}

void beacon_meteo_status() {
  if (cnt_meteo_send == 0 )
    {
      String meteoStatus = String(METEO_CALLSIGN) + ">" + String(DESTCALL) + ":>" + String(METEO_STATUS);
      aprsis_send(meteoStatus);
    }  
}

void beacon_upload() {
  lastUpload = millis();
  if (Use_UPLOAD) upload_data(String(getTempC()) + "," + String(int(getHum())) + "," + String(int(getPressure())) + "," + String(windActualSpeed) + "," + String(windLongPeriodSpeed) + "," + String(voltage) + "," + String(gust));
}

bool check_wifi() {
  if (WiFi.status() == WL_CONNECTED)
    return true;
  else
    return false;
}

bool check_aprsis() {
  if (aprsis.connected() && check_wifi())
    return true;
  else
    return false;
}


float getTempC()
{
  if (BM_sensor_status || AHTstatus )
        
    if (ch_term == 0 && BMPstatus ) return bmp.readTemperature()+drift_therm;  // use BMP280
    if ( BMEstatus ) 
      {
      //sensors_event_t temp_event, pressure_event, humidity_event;
      bme_temp->getEvent(&temp_event);
      return temp_event.temperature;+drift_therm;  // use BME280
      
      }
    if (ch_term == 1 )                                                                   // use AHT20  
      {
       
        //sensors_event_t humidity_event, temp_event;
        aht.getEvent(&humidity_event, &temp_event);
        return temp_event.temperature+drift_therm;
      }
  
  else return 0;
}

float getHum()
  {
    if (AHTstatus || BMEstatus)
   
        if(AHTstatus)
          {
            aht.getEvent(&humidity_event, &temp_event);
            return humidity_event.relative_humidity;
          }
        if(BMEstatus) 
          {
            bme_humidity->getEvent(&humidity_event);
            return humidity_event.relative_humidity;
          }
     
    else return 0;
  }



float getPressure()
  {
    if (BM_sensor_status)

    if(BMPstatus) return( (bmp.readPressure()           * ( pow(1.0 - (0.0065 * atoi(altitude) * -1 / (273.15 + getTempC() )), 5.255)) ) / 100 );
    if(BMEstatus)
      {
        bme_pressure->getEvent(&pressure_event);
                  return( ((pressure_event.pressure*100) * ( pow(1.0 - (0.0065 * atoi(altitude) * -1 / (273.15 + getTempC() )), 5.255)) ) / 100 );
      }
    
    else return 0;
  }

  
String getHumAPRS() {
  if (AHTstatus == true || BMEstatus == true ) {
    String sHum = String(int(getHum()));
    return sHum;
  } else {
    return "..";
  }
}



String getTempAPRS() {
  if (BM_sensor_status) {
  float fahrenheit = getTempC();
  fahrenheit *= 9;
  fahrenheit /= 5;
  int ifahrenheit = fahrenheit + 32;
  if (!(ifahrenheit < 1000)) ifahrenheit = 0;
  String sfahrenheit = String(ifahrenheit);
  if (ifahrenheit < 100) sfahrenheit = String("0") + String(sfahrenheit);
  if (ifahrenheit < 10) sfahrenheit = String("0") + String(sfahrenheit);
  return sfahrenheit;
  } else return "000";
}


String getPressureAPRS() {
  if (BM_sensor_status) {
  //float press = getPressure();
  //press *= 10;
  
  float tmp_press = getPressure();
  int press = (tmp_press*10);
  if (press > 99999) press = 0;
  String spress = String(press);
  if (press < 10000) spress = String("0") + String(spress);
  if (press < 1000) spress = String("0") + String(spress);
  if (press < 100) spress = String("0") + String(spress);
  if (press < 10) spress = String("0") + String(spress);
  return spress;
  } else {
    return "00000";
  }
}


void hall_change() {
  windMeterSpins++;
  windMeterSpinsInTimeout++;
}

float mph(float metersPerSeconds) {
  return metersPerSeconds * 2.23693629;
}

String windSpeedAPRS(float fSpeed) {
  if (USE_ANEMOMETER) {
    int wSpeed = int(mph(fSpeed));
    if (wSpeed >= 1000) wSpeed = 0;
    String sSpeed = String(wSpeed);
    if (wSpeed < 100) sSpeed = "0" + String(sSpeed);
    if (wSpeed < 10) sSpeed = "0" + String(sSpeed);
    return sSpeed;
  } else {
    return "...";
  }
}


void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len){
  if(type == WS_EVT_CONNECT){
    Serial.println("WS Con: " + String(ws.count()) + " connected.");
    isWSconnected = true;
  } else if(type == WS_EVT_DISCONNECT){
    Serial.println("WS Dis: " + String(ws.count()) + " connected.");
    if(ws.count() == 0) isWSconnected = false;
  }
}


void updateWebSocket() {
  ws.textAll(tempToWeb(getTempC()) + "," + HumToWeb(getHum()) + "," + pressToWeb(getPressure()) + "," + windToWeb(windActualSpeed) + "," + windToWeb(windKMH(windActualSpeed)) + "," + windToWeb(gust) + "," + windToWeb(windLongPeriodSpeed));
  lastWSupdate = millis();
  //Serial.println("WS: Updated.");
}


String HTMLelementDef(String elementID) {
  return " var " + elementID + " = document.getElementById('" + elementID + "'); ";
}

String ipToString(IPAddress ip){
  String s="";
  for (int i=0; i<4; i++)
    s += i  ? "." + String(ip[i]) : String(ip[i]);
  return s;
}

float windKMH(float windMS) {
  return windMS * 3.6;
}

String tempToWeb(float tempValue) {
  if (BM_sensor_status) return String(tempValue,1);
  else return "N/A";
}

String HumToWeb(float HumValue) {
  if (AHTstatus == true || BMEstatus == true ) return String(HumValue,1);
  else return "N/A";
}

String pressToWeb(float pressValue) {
  if (BM_sensor_status) return String(pressValue,1);
  else return "N/A";
}

String windToWeb(float windValue) {
  if (USE_ANEMOMETER) return String(windValue);
  else return "N/A";
}

String valueForJSON(String value) {
  if (value == "N/A")
    return "null";
  else
    return value;
}

String addGraphValue(String values, String value) {
  int count = 0;
  char searchChar = ',';
  for (int i = 0; i < values.length(); i++) {
    if (values[i] == searchChar) {
      count++;
    }
  }
  if (count > int(GRAPH_LIMIT) - 2)
    values = values.substring(values.indexOf(",") + 1);
  if (values != "") values += ",";
  values += valueForJSON(value);
  return values;
}

String generateGraph(String values, String graphName, String graphID, int r, int g, int b) {
  String graphScript = "<b style='width: 100%; text-align: center'>" + graphName +"</b><br><br><canvas id='" + graphID + "' style='width:100%'></canvas> \
  <script>var yValues = [" + values + "]; \
  var xValues = [";

  int count = 0;
  char searchChar = ',';
  for (int i = 0; i < values.length(); i++) {
    if (values[i] == searchChar) {
      count++;
    }
  }
  for (int i = 0; i <= count; i++) {
    graphScript += "' '";
    if (i != count) graphScript += ",";
  }
  
  graphScript += "];";
  if (values != "") {
    graphScript += "var " + graphID + "_min = Math.min(...yValues); var " + graphID + "_max = Math.max(...yValues);";
  } else {
    graphScript += "var " + graphID + "_min = 0; var " + graphID + "_max = 0;";
  }
  graphScript += "new Chart('" + graphID + "', {\
  type: 'line',\
  data: {\
    labels: xValues,\
    datasets: [{\
      fill: false,\
      lineTension: 0,\
      backgroundColor: 'rgba(";
  graphScript += String(r) + "," + String(g) + "," + String(b);
  graphScript += ",1.0)',";
  graphScript += "borderColor: 'rgba(";
  graphScript += String(r) + "," + String(g) + "," + String(b);
  graphScript += ",0.1)',\
      data: yValues\
    }]\
  },\
  options: {\
    legend: {display: false},\
    scales: {\
      yAxes: [{ticks: {min: " + graphID + "_min, max: " + graphID + "_max}}]\
    }\
  }\
  });\
  </script><br>";
  return graphScript;
}

bool GETIndex(String header, String requestPath) {
  if (header.indexOf("GET " + requestPath) >= 0)
    return true;
  else
    return false;
}

bool getBM_sensor_status() {
  return BM_sensor_status;
}

bool getAHTstatus() {
  return AHTstatus;
}

bool getBMEstatus() {
  return BMEstatus;
}



 /*
  ---------------------------------------------------------------------------
    FUNZIONE LETTURA DIGITAZIONI DA MENU 
  ---------------------------------------------------------------------------
  */
  
int readCharArray(char *tmp_buffer)
{
  char car;
  sep=0;
  ptr=0;
  do
    {
      if (Serial.available() > 0)
        {
          car = Serial.read();
          
          if ( carMenu == 'c'  )       // per il menu 1 converti in maiuscolo
            {  
              if ( car >= 97 && car <= 122 ) car = car-32;  // converti da minuscolo a maiuscolo
            }
          
          if (car != '\n')
            {
              if ( carMenu == '2' || carMenu == '6' )
                {
                  if ( car != ' ' )  // lat lon vengono eliminati tutti gli spazi
                    {
                      tmp_buffer[ptr++] = car;
                      if (car == ',') sep=ptr-1;    // posizione del separatore lat/lon
                    } 
                }    
              else tmp_buffer[ptr++] = car;       
             }
          }    
      }
  while (car != '\n');
  
  tmp_buffer[ptr] = 0;
  // return the number of char read
  return ptr;
  return sep;
}



  /*
  ---------------------------------------------------------------------------
    LETTURA CARATTERE SCELTA MENU 
  ---------------------------------------------------------------------------
  */

char readCarMenu()
{
  //char car = 0;
  char ret = 0;
  ret = 0;
  while (car != '\n')
    {
      if (Serial.available() > 0)
        {
          car = Serial.read();
          if ((car >= 48) && (car <= 57) || (car >= 97) && (tmp <= 122) ) ret = car;
        }
    }
  car=0;
  return ret;
}




 /*
  ---------------------------------------------------------------------------
    MENU 
  ---------------------------------------------------------------------------
  */
 void menu()

{
  do
    {
      carMenu = 0;
      righello();
      Serial.println(F("\n.. CONFIG MENU ..\n"));
      righello();
      Serial.println(F("(c) callsign"));
      righello();
      Serial.println(F("(1) meteo ssid"));
      Serial.println(F("(2) meteo lat/long"));
      Serial.println(F("(a) meteo altitude"));
      if ( AHTstatus == true ) Serial.println(F("(u) meteo use thermometer"));
      Serial.println(F("(3) meteo drift thermal sensor"));
      Serial.println(F("(4) meteo info"));
      Serial.println(F("(t) meteo beacon tx interval"));
      Serial.println(F("(m) meteo APRS-IS switch"));
      righello();
      Serial.println(F("(5) iGate ssid"));
      Serial.println(F("(6) iGate lat/long"));
      Serial.println(F("(i) iGate APRS-IS switch"));
      Serial.println(F("(d) digipeater switch"));
      //Serial.println(F("(6) igate info"));
      righello();
           
      Serial.println(F("(f) LoRa frequency"));
      Serial.println(F("(p) LoRa power"));
      Serial.println(F("(w) Wifi ssid"));
      Serial.println(F("(7) Wifi password"));
      Serial.println(F("(8) APRS passcode"));
      Serial.println(F("(9) APRS server"));
      
      Serial.println(F("(0) EXIT"));
      righello();
    
      carMenu = readCarMenu();
      switch (carMenu)
        {
          case '0' :
           status_display();
           break;

          case 'c' :
            Serial.print(F("callsign - ex IZ1XYZ"));
            readCharArray(call);
         
            EEPROM_writer(6,ptr,call);
            EEPROM_eraser(6+ptr,11);

            Serial.print(F(" = "));
            Serial.println(call);
            break;


            case 'd' :
            Serial.print(F("0=disabled | 1=enabled - ex: 1"));
            readCharArray(tmp_buffer);
            if (atoi(tmp_buffer) > 0 ) digiSwitch = true;
            else digiSwitch = false;
            Serial.print(F(" = "));
            Serial.print(tmp_buffer);
            if ( digiSwitch == false ) Serial.println(F(" = DISABLED"));
            if ( digiSwitch == true ) Serial.println(F(" = ENABLED"));
            EEPROM.write(167, atoi(tmp_buffer));
            EEPROM.commit();
           break; 
          
          
          case 'i' :
            Serial.print(F("0=disabled | 1=enabled - ex: 1"));
            readCharArray(tmp_buffer);
            if (atoi(tmp_buffer) > 0 ) aprsSwitch = true;
            else aprsSwitch = false;
            Serial.print(F(" = "));
            Serial.print(tmp_buffer);
            if ( aprsSwitch == false ) Serial.println(F(" = DISABLED"));
            if ( aprsSwitch == true ) Serial.println(F(" = ENABLED"));
            EEPROM.write(166, atoi(tmp_buffer));
            EEPROM.commit();
           break;  
          
          
            case 'm' :
            Serial.print(F("0=disabled | 1=enabled - ex: 1"));
            readCharArray(tmp_buffer);
            if (atoi(tmp_buffer) > 0 )meteoSwitch = true;
            else meteoSwitch = false;
            Serial.print(F(" = "));
            Serial.print(tmp_buffer);
            if ( meteoSwitch == false ) Serial.println(F(" = DISABLED"));
            if ( meteoSwitch == true ) Serial.println(F(" = ENABLED"));
            EEPROM.write(165, atoi(tmp_buffer));
            EEPROM.commit();
           break;  
           

          case 'u' :
            if ( AHTstatus == false ) break;
            Serial.print(F("0=use BMP280 | 1=use AHT20 - ex: 0"));
            readCharArray(tmp_buffer);
            ch_term = atoi(tmp_buffer);
            verifica_parametri();
            Serial.print(F(" = "));
            Serial.print(ch_term);
            if ( ch_term == 0 ) Serial.println(F(" = BMP280"));
            if ( ch_term == 1 ) Serial.println(F(" = AHT20"));
            EEPROM.write(52, ch_term);
            EEPROM.commit();
           break;          
          
 
          case '1' :
            Serial.print(F("meteo ssid - ex: 3"));
            readCharArray(tmp_buffer);
            meteo_ssiD = atoi(tmp_buffer);
            verifica_parametri();
            Serial.print(F(" = "));
            Serial.println(meteo_ssiD);
            EEPROM.write(12, meteo_ssiD);
            EEPROM.commit();
           break;  


          case '5' :
            Serial.print(F("igate ssid - ex: 10"));
            readCharArray(tmp_buffer);
            igate_ssiD = atoi(tmp_buffer);
            verifica_parametri();
            Serial.print(F(" = "));
            Serial.println(igate_ssiD);
            EEPROM.write(13, igate_ssiD);
            EEPROM.commit();
           break;   
  

          case '2' :
            Serial.print(F("lat,long meteo - ex: 78.7562,18.5162"));
            readCharArray(tmp_buffer);
        
            if ( ptr > 22 || sep == 0)
              {
                Serial.print(F(" .. error .. \n"));
                break;
              }            

            array_eraser(0,9,lat_meteo);
            array_eraser(0,10,lon_meteo);
            EEPROM_eraser(14,34);

            tmp=14;
            while (tmp != 14+sep) // -- salva lat meteo
              {
                  EEPROM.write( tmp, tmp_buffer[tmp-14] ); // si scrive a partire dalla cella 13 fino alla 22
                  EEPROM.commit();
                  delay(30);
                  lat_meteo[tmp-14]=tmp_buffer[tmp-14];
                  tmp++;
              } 

            tmp=24;
            while (tmp != 24+ptr-sep-1) // -- salva lon meteo
              {
                  EEPROM.write( tmp, tmp_buffer[tmp-24+sep+1] ); // si scrive a partire dalla cella 23 fino alla 33
                  EEPROM.commit();
                  delay(30);
                  lon_meteo[tmp-24]=tmp_buffer[tmp-24+sep+1];
                  tmp++;
              } 
       
            Serial.print(F(" = "));
            Serial.print(atof(lat_meteo),6);
            Serial.print(F(","));
            Serial.println(atof(lon_meteo),6);
            verifica_parametri();         // calcola in notazione APRS
            break; 
            

           case '6' :
            Serial.print(F("lat,long igate - ex: 78.7562,18.5162"));
            readCharArray(tmp_buffer);
        
            if ( ptr > 22 || sep == 0)
              {
                Serial.println(F(" .. error .. \n"));
                break;
              }            

            array_eraser(0,9,lat_igate);
            array_eraser(0,10,lon_igate);
            EEPROM_eraser(230,250);

            tmp=230;
            while (tmp != 230+sep) // -- salva lat igate
              {
                  EEPROM.write( tmp, tmp_buffer[tmp-230] ); // si scrive a partire dalla cella 230 fino alla 239
                  EEPROM.commit();
                  delay(30);
                  lat_igate[tmp-230]=tmp_buffer[tmp-230];
                  tmp++;
              } 
           
            tmp=240;
            while (tmp != 240+ptr-sep-1) // -- salva lon igate
              {
                  EEPROM.write( tmp, tmp_buffer[tmp-240+sep+1] ); // si scrive a partire dalla cella 240 fino alla 250
                  EEPROM.commit();
                  delay(30);
                  lon_igate[tmp-240]=tmp_buffer[tmp-240+sep+1];
                  tmp++;
              }           
                             
            Serial.print(F(" = "));
            Serial.print(atof(lat_igate),6);
            Serial.print(F(","));
            Serial.println(atof(lon_igate),6);
            verifica_parametri();         // calcola in notazione APRS

            break;
      
          
          case 'a' :
            Serial.print(F("altitude - ex: 445"));
            readCharArray(altitude);
            if (ptr > 4) ptr = 4;
            EEPROM_writer(35,ptr,altitude);
            EEPROM_eraser(35+ptr,38);
            Serial.print(F(" = "));
            Serial.println(altitude);
            break;


            case 'w' :
            Serial.print(F("WiFi ssiD [ max 20 char ]"));
            readCharArray(WiFi_ssiD);
            if ( ptr>20 ) ptr=20; 
            EEPROM_writer(170,ptr,WiFi_ssiD);
            EEPROM.write( 170+ptr, 254 );   // aggiungo flag di fine testo
            EEPROM.commit();
            Serial.print(F(" = "));
            Serial.println(WiFi_ssiD);
            break;


            case '7' :
            Serial.print(F("WiFi password [ max 20 char ]"));
            readCharArray(WiFi_pwd);
            if (ptr>20 ) ptr=20;
            EEPROM_writer(190,ptr,WiFi_pwd);
            EEPROM.write( 190+ptr, 254 );   // aggiungo flag di fine testo
            EEPROM.commit();
            Serial.print(F(" = "));
            Serial.println(WiFi_pwd);
            break;


            case '8' :
            Serial.print(F("APRS passcode - ex: 19617"));
            readCharArray(aprs_passcode);
            EEPROM_writer(53,53+4,aprs_passcode);
            Serial.print(F(" = "));
            Serial.println(aprs_passcode);
            break;


            case '9' :
            Serial.print(F("APRS server - ex: rotate.aprs2.net"));
            readCharArray(aprs_server);
            if (ptr>20 ) ptr=20;
            EEPROM_writer(210,ptr,aprs_server);
            EEPROM.write( 210+ptr, 254 );   // aggiungo flag di fine testo
            EEPROM.commit();
            Serial.print(F(" = "));
            Serial.println(aprs_server);
            APRSISServer = String(aprs_server);
            break;

        
          case '4' :
            Serial.print(F("meteo info - ex: .. WXmeteo LoRa tech .."));
            readCharArray(meteo_info);
            if (ptr>50 ) ptr=50;
            EEPROM_writer(60,ptr,meteo_info);
            EEPROM.write( 60+ptr, 254 );   // aggiungo flag di fine testo
            EEPROM.commit();
            Serial.print(F(" = "));
            Serial.println(meteo_info);
            break;

          /*
          case 'x' :
            Serial.print(F("igate info - ex: .. iGate LoRa tech .."));
            readCharArray(igate_info);
            if (ptr>50 ) ptr=50;
            EEPROM_writer(112,ptr,igate_info);
            EEPROM.write( 112+ptr, 254 );   // aggiungo flag di fine testo
            EEPROM.commit();
            Serial.print(F(" = "));
            Serial.println(igate_info);
            break;
          */

          case 'f' :
            Serial.print(F("frequency - ex: 433775"));
            readCharArray(frequencyC);
            if (ptr>6) ptr=6;
            EEPROM_writer(41,ptr,frequencyC);
            Serial.print(F(" = "));  
            Serial.println(frequencyC);
            break;

          case '3' :
            Serial.print(F("Thermal drift (max +/- 5) | ex: -0.25"));
            readCharArray(drift_thermC);
            if (atof(drift_thermC) >5 || atof(drift_thermC) < -5 )
              {
                Serial.println(F(" = ERROR"));
                break;
              }
            EEPROM_eraser(47,51);       // cancello dalla 47 alla 51
            if (ptr>5 ) ptr=5;
            EEPROM_writer(47,ptr,drift_thermC);
            Serial.print(F(" = "));
            drift_therm = atof(drift_thermC);  
            Serial.println(drift_therm, 2);   
            break;


          case 'p' :
            Serial.print(F("power 2-17 dBm - ex: 17"));
            readCharArray(tmp_buffer);
            LoRa_power = atoi(tmp_buffer);
            verifica_parametri();
            Serial.print(F(" = "));
            Serial.println(LoRa_power);
            EEPROM.write(38, LoRa_power);
            EEPROM.commit();
            break;


          case 't' :
            Serial.print(F("tx interval minutes - ex: 10"));
            readCharArray(tmp_buffer);
            tx_interval = atoi(tmp_buffer);
            verifica_parametri();
            Serial.print(F(" = "));
            Serial.println(tx_interval);
            EEPROM.write(40, tx_interval);
            EEPROM.commit();
            break;
         
         }        
  } while (carMenu != '0' );
  Serial.println();
}

 /*
  ---------------------------------------------------------------------------
    RIGHELLO 
  ---------------------------------------------------------------------------
  */

void righello()
{
  tmp=0;
  while (tmp != 60)
    {
      Serial.print(F("- "));
      tmp++;  
    }
    Serial.println(F("-"));
    tmp=0;
}

void banner()
  {
    Serial.print(F("\n.. "));
    Serial.print(F(Project));
    Serial.print(F(" "));
    Serial.print(F(Release));
    Serial.print(F(" - built "));
    Serial.println(F(Build));
    Serial.println(F("   based on OK2DDS' project modified by IW1CGW..\n"));
  }

 /*
  ---------------------------------------------------------------------------
    STATUS SERIAL DISPLAY 
  ---------------------------------------------------------------------------
  */

void status_display()
  {
    righello();
    banner();
    righello();

    Serial.print(F("meteo call: "));
    Serial.print(call);
    Serial.print(F("-"));
    Serial.println(meteo_ssiD);
    
    Serial.print(F("meteo lat,long: "));
    Serial.print(atof(lat_meteo),6);
    Serial.print(F(" , "));
    Serial.println(atof(lon_meteo),6);
    
    Serial.print(F("meteo info: "));
    Serial.println(meteo_info);
    /*
    Serial.print(F(" - "));
    Serial.print(lat_meteo_APRS);
    Serial.print(F("/"));
    Serial.println(lon_meteo_APRS);
    */
    Serial.print(F("meteo station altitude: ")); Serial.print(atoi(altitude));Serial.println(F(" m"));
        if ( AHTstatus == true )
      {
        Serial.print(F("thermometer in use: "));
        if (ch_term == 0 ) Serial.println(F("BMP280"));
        if (ch_term == 1 ) Serial.println(F("AHT20"));
       }
    Serial.print(F("thermal drift: ")); Serial.print(drift_therm);  Serial.println(F(" Celsius"));
    Serial.print(F("meteo beacon tx interval: ")); Serial.print(tx_interval); Serial.println(F(" minutes"));
    Serial.print(F("meteo APRS-IS: "));
    if (meteoSwitch) Serial.println(F("enabled"));
    else Serial.println(F("disabled"));
    righello();
    
    Serial.print(F("iGate call: "));
    Serial.print(call);
    Serial.print(F("-"));
    Serial.println(igate_ssiD);

    Serial.print(F("iGate lat,long: "));
    Serial.print(atof(lat_igate),6);
    Serial.print(F(" , "));
    Serial.println(atof(lon_igate),6);

    Serial.print(F("iGate APRS-IS; "));
    if (aprsSwitch ) Serial.println(F("enabled"));
    else Serial.println(F("disabled"));

    Serial.print(F("digipeater: "));
    if (digiSwitch ) Serial.println(F("enabled"));
    else Serial.println(F("disabled"));

    /*
    Serial.print(F(" - "));
    Serial.print(lat_igate_APRS);
    Serial.print(F("/"));
    Serial.println(lon_igate_APRS);
    */
    //Serial.print(F("iGate info: ")); Serial.println(igate_info);
    
    righello();
    Serial.print(F("LoRa frequency: ")); Serial.print(frequencyC); Serial.println(F(" KHz"));
    Serial.print(F("LoRa power: ")); Serial.print(LoRa_power); Serial.println(F(" dBm"));
    Serial.print(F("Wifi ssid: ")); Serial.println(WiFi_ssiD);
    Serial.print(F("Wifi password: "));  Serial.println(WiFi_pwd);
    Serial.print(F("APRS passcode for ")); Serial.print(call); Serial.print(F(": "));Serial.println(aprs_passcode);
    Serial.print(F("APRS server: ")); Serial.println(aprs_server);

    righello();
    Serial.println(F(bottom));

  }

  

  /*
  ---------------------------------------------------------------------------
    CARICAMENTO PARAMETRI
  ---------------------------------------------------------------------------
  */

void load_param()
  {
    /* ---------------------- determina se occorre factory reset 
      il primi 6 caratteri della EEPROM si attendono il dato della Build
      se non coincide cosa si legge in EEPROM viene avviata la routine di factory reset
    */ 

    boolean verify = LOW;
    tmp=0;
    while ( tmp != 6  )       // leggi dalla cella 0 fino alla cella 6 
      {
        car = EEPROM.read(tmp);
        if (car != Build[tmp] ) verify = true; // verificare da scansione se Build e build_mark sono uguali
        tmp++;
      }

    if ( verify == true )
      {
        Serial.println(F("initial reset required  - please wait")); 
        initial_reset();        // carica valori di default nella EEPROM
      }
 
    EEPROM_loader(6,11,call);
    meteo_ssiD = EEPROM.read( 12 );         
    igate_ssiD = EEPROM.read( 13 );         
    EEPROM_loader(14,23,lat_meteo);
    EEPROM_loader(24,34,lon_meteo);
    EEPROM_loader(35,38,altitude);
    LoRa_power = EEPROM.read( 39 ); 
    tx_interval = EEPROM.read( 40 ); 
    EEPROM_loader(41,46,frequencyC);
    EEPROM_loader(47,51,drift_thermC);
    ch_term = EEPROM.read( 52 ); 
    EEPROM_loader(53,57,aprs_passcode);
    
    if (EEPROM.read( 165 ) > 0 ) meteoSwitch = true;  // USE_METEO
    else meteoSwitch = false;
    
    if (EEPROM.read( 166 ) > 0 ) aprsSwitch = true;   // Use_IGATE
    else aprsSwitch = false;
    
    if (EEPROM.read( 167 ) > 0 ) digiSwitch = true;   // USE_DIGIPEATER
    else digiSwitch  = false;

    EEPROM_loader(60,111,meteo_info);
    //EEPROM_loader(112,163,igate_info);

    EEPROM_loader(170,189,WiFi_ssiD);
    EEPROM_loader(190,209,WiFi_pwd);
    EEPROM_loader(210,229,aprs_server);
    EEPROM_loader(230,239,lat_igate);
    EEPROM_loader(240,250,lon_igate);
    verifica_parametri();
  }


  /*
  ---------------------------------------------------------------------------
    RESET INIZIALE
  ---------------------------------------------------------------------------
  */

void initial_reset()
  {
    
    EEPROM_eraser(0,250);
    Serial.println(F("EEPROM erased - please wait"));

    EEPROM_writer(0, 5,Build);   
    char buff0[10]="N0CALL"; 
    EEPROM_writer(6, 11,buff0);
    char buff1[10]="25"; 
    EEPROM_writer(35, 38,buff1);

    EEPROM_writer(41, 46,frequencyC);       // frequenza dalla cella 41 fino alla 46
    char buff2[6]="12345";                  // buff2 = 5 char [ 0--> 4 ]
    EEPROM_writer(53, 57,buff2);
    EEPROM_writer(53, 57,buff2);
    char buff3[24]=".. WXmeteo LoRa tech .."; // buff3 = 23 char [ 0--> 22 ]

    EEPROM_writer(60,   60+22,buff3);EEPROM.write(60+23,  254);   // buff3 = 23 char [ 0--> 22 ]
    EEPROM_writer(112, 112+22,buff3);EEPROM.write(112+23, 254);   // buff3 = 23 char [ 0--> 22 ]
    EEPROM_writer(170,  170+4,buff2);EEPROM.write(170+5,  254);   // buff2 = 5 char [ 0--> 4 ]
    EEPROM_writer(190,  190+4,buff2);EEPROM.write(190+5,  254);   // buff2 = 5 char [ 0--> 4 ]
    EEPROM_writer(210,  210+4,buff2);EEPROM.write(210+5,  254);   // buff2 = 5 char [ 0--> 4 ]

    EEPROM.write(12, 3);
    EEPROM.write(13, 10);   
    EEPROM.write(39, 2);
    EEPROM.write(40, 10);
    EEPROM.write(165, 1);// switch meteo
    EEPROM.write(166, 1);// switch igate
    EEPROM.write(167, 1);// switch digi

    EEPROM.commit();

   Serial.println(F("initial reset executed  - please wait")); 
    
}


  /*
  ---------------------------------------------------------------------------
    VERIFICA PARAMETRI
  ---------------------------------------------------------------------------
  */

void verifica_parametri()
  {
    tmp=0;
    
    if (meteo_ssiD >15)   meteo_ssiD = 3;
    if (meteo_ssiD <1)    meteo_ssiD = 3;
    if (igate_ssiD >15)   igate_ssiD = 10;
    if (igate_ssiD <1)    igate_ssiD = 10;
    if (LoRa_power <2 )   LoRa_power = 2;
    if (LoRa_power >17 )  LoRa_power = 17;
    if (tx_interval <1 )  tx_interval = 10;
    if (tx_interval >20 ) tx_interval = 20;
    
    //hum_lect = time_data_period/(tx_interval*60000);
    //Serial.print("hum_lect: ");
    //Serial.println(hum_lect);

    drift_therm=atof(drift_thermC);
    if ( drift_therm > 5 ) drift_therm = 0;
    if ( drift_therm < -5 ) drift_therm = 0;

    if ( ch_term >1 ) ch_term = 0;
    if ( AHTstatus == false ) ch_term = 0;

    if ( atoi(lon_igate) >90 || atoi(lat_igate) >180 ) 
      {
        array_eraser(0,9,lat_igate);
        array_eraser(0,10,lon_igate);
        Serial.print(F("\n .. error lat/lon igate .. \n"));
      } 
  
   if ( atoi(lon_meteo) >90 || atoi(lat_meteo) >180 ) 
      {
        array_eraser(0,9,lat_meteo);
        array_eraser(0,10,lon_meteo);
        Serial.print(F("\n .. error lat/lon meteo .. \n"));
      } 
  
    METEO_CALLSIGN = String(call)+String("-")+String(meteo_ssiD);
    IGATE_CALLSIGN = String(call)+String("-")+String(igate_ssiD); 
    APRSISServer = String(aprs_server);
    APRS_LatLon();

    if (!aprsSwitch) {
      LoRa.sleep();
    }

  }


void APRS_LatLon()
 {
  //-----------------------------------------------
  if (lat_meteo[0] == '-' ) lat_meteo[0] = ' ' ;
  int gradi=atoi(lat_meteo);
  double minuti=(atof(lat_meteo)-gradi)*60;
  double secondi = (minuti-int(minuti))*100;
  float centesimi = (secondi-int(secondi))*100;
  if (centesimi > 50 ) secondi = secondi+1;
  if (lat_meteo[0] == ' ' ) lat_meteo[0] = '-' ;  

  sprintf(lat_meteo_APRS, "%02d%02d.%02dN",(gradi),(int(minuti)),(int(secondi)));    
  if (lat_meteo[0] == '-' ) sprintf(lat_meteo_APRS, "%02d%02d.%02dS",(gradi),(int(minuti)),(int(secondi)));  
  //-----------------------------------------------
  
  //-----------------------------------------------
  if (lon_meteo[0] == '-' ) lon_meteo[0] = ' ' ;
  gradi=atoi(lon_meteo);
  minuti=(atof(lon_meteo)-gradi)*60;
  secondi = (minuti-int(minuti))*100;
  centesimi = (secondi-int(secondi))*100;
  if (centesimi > 50 ) secondi = secondi+1;
  if (lon_meteo[0] == ' ' ) lon_meteo[0] = '-' ; 
  sprintf(lon_meteo_APRS, "%03d%02d.%02dE",(gradi),(int(minuti)),(int(secondi)));  
  if (lon_meteo[0] == '-' ) sprintf(lon_meteo_APRS, "%03d%02d.%02dW",(gradi),(int(minuti)),(int(secondi)));  
  //-----------------------------------------------

  //-----------------------------------------------
    if (lat_igate[0] == '-' ) lat_igate[0] = ' ' ;
    gradi=atoi(lat_igate);
    minuti=(atof(lat_igate)-gradi)*60;
    secondi = (minuti-int(minuti))*100;
    centesimi = (secondi-int(secondi))*100;
    if (centesimi > 50 ) secondi = secondi+1;
    if (lat_igate[0] == ' ' ) lat_igate[0] = '-' ;  

    sprintf(lat_igate_APRS, "%02d%02d.%02dN",(gradi),(int(minuti)),(int(secondi)));    
    if (lat_igate[0] == '-' ) sprintf(lat_igate_APRS, "%02d%02d.%02dS",(gradi),(int(minuti)),(int(secondi)));  
    //-----------------------------------------------
    
    //-----------------------------------------------
    if (lon_igate[0] == '-' ) lon_igate[0] = ' ' ;
    gradi=atoi(lon_igate);
    minuti=(atof(lon_igate)-gradi)*60;
    secondi = (minuti-int(minuti))*100;
    centesimi = (secondi-int(secondi))*100;
    if (centesimi > 50 ) secondi = secondi+1;
    if (lon_igate[0] == ' ' ) lon_igate[0] = '-' ; 
    sprintf(lon_igate_APRS, "%03d%02d.%02dE",(gradi),(int(minuti)),(int(secondi)));  
    if (lon_igate[0] == '-' ) sprintf(lon_igate_APRS, "%03d%02d.%02dW",(gradi),(int(minuti)),(int(secondi)));  
    //-----------------------------------------------

 }


void array_eraser(byte start , byte stop , char tmp_data[50])
{
  while (start <= stop) // -- cancella array lat/lon igate
    {
      tmp_data[start] = ' '; 
      start++;
    }
}


void EEPROM_eraser(byte start,byte stop)
  {
    while (start <= stop) // -- cancella EEPROM 
      {
        EEPROM.write( start, ' ' ); 
        start++;
      }
    EEPROM.commit(); 
  }


void EEPROM_writer(byte start, byte stop,char tmp_data[50])
  {
    tmp=0;
    while (tmp+start <= stop) // -- scrivi EEPROM 
      {
        EEPROM.write( tmp+start, tmp_data[tmp] ); 
        tmp++;
      }
    EEPROM.commit(); 
  }


 void EEPROM_loader(byte start, byte stop,char tmp_data[50])
  {
    tmp=0;
    while ( tmp+start <= stop )       
      {
        car = EEPROM.read(tmp+start);
        if ( car == 254 ) return; 
        tmp_data[tmp] = car;
        tmp++;
      }
        
  }



 /*
  ---------------------------------------------------------------------------
    EEPROM MAP
  ---------------------------------------------------------------------------


  byte of the EEPROM can only hold a
  value from 0 to 255.
 
  
0 - 5   // Build:230405 | 6
6 - 11  // call: IZ1GZA | 6
12 - 12 // meteo_iD: 13 | 1  
13 - 13 // igate_iD: 13 | 1  
14 - 23 // latitude meteo | 10 caratteri
24 - 34 // longitudine meteo 11 caratteri 
35 - 38 // altitude - 1000 | 4
39 - 39 // set power: 2-17 | 1
40 - 40 // set tx_interval 1-20 | 10
41 - 46 // frequenza | 6 caratteri ex: 433775
47 - 51 // drift termometro | 5 caratteri
52 - 52 // scelta termometro [ solo per BMP280+AHT20: 0= temp BMP280 | 1= temp AHT20
53 - 57 // aprs passcode

60 - 111 // info meteo: .. WXmeteo Alma Frabosa - 650m .. | 50 caratteri + flag fine testo - carattere cella 111 -
112 - 163 // info igate: .. WXmeteo Alma Frabosa - 650m .. | 50 caratteri + flag fine testo - carattere cella 163 -

165 - 165 // switch meteo [meteoSwitch]
166 - 166 / /switch igate [aprsSwitch]
167 - 167 // switch digipeater [digiSWitch]

170 - 189 // WiFi ssiD
190 - 209 // WiFi password
210 - 229 // Aprs_server

230 - 239 // latitude igate | 10 caratteri
240 - 250 // longitudine igate |11 caratteri 

 */