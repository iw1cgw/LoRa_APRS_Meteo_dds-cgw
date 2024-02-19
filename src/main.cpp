#include <Arduino.h>
#include "config.h"
#include <LoRa.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <Update.h>
//#include <ArduinoJson.h>
#include <ESP32_FTPClient.h>
#include <HTTPClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_BME280.h>
#include <Adafruit_AHTX0.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "website.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>//https://github.com/espressif/arduino-esp32/tree/master/libraries/EEPROM

/*
bool vpn();

#include <Husarnet.h>
const char *hostName = "esp32-webserver";
const char *husarnetJoinCode = "...............";

*/

void lora_setup();
void lora_send(String tx_data);
void aprsis_connect();
void aprsis_send(String aprsis_packet);
void beacon_igate();
void beacon_meteo();
void beacon_meteo_status();
void beacon_igate_status();


bool check_wifi();
bool check_aprsis();


bool checkForUpdates();
bool updateFirmware();
bool token_verify_update;

void OTA_display_ko();
void OTA_logbook();
bool NTP_query();
struct tm timeinfo;
char NTP_data[20];
uint8_t OTA_code;

unsigned long millis_token_tx;
bool token_tx;
String Tmp;
uint8_t retr;

void start_server();
WiFiServer server(80);
WiFiClient aprsis;
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

static time_t aprsLastReconnect = 0;
static time_t lastIgBeacon = 0;
static time_t lastStIgBeacon = 0;   // timer del beacon status dell'igate 
static time_t lastMtBeacon = 0;
static time_t lastStBeacon = 0;
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

uint8_t meteo_tx_mode = 0;      // meteo_tx_mode | 1 = in APRS-IS [meteoAPRSswitch] | 2 = in LoRa [meteoSwitch] | 0  = disable
bool APRS_login = false;
bool igateSwitch = false;       // = USE iGate
bool digiSwitch = false;        // = USE_DIGIPEATER
bool backupigateSwitch = false; // backup su fault igate, inserisce digipeatyer
bool oledSwitch = true;         // = oled acceso
bool Use_WiFi;                  // value loaded from EEPROM
bool wifiStatus = false;

#define SSD1306_ADDRESS 0x3C
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET 23 
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
const unsigned char logo [] PROGMEM = {
// '128X64', 128x64px
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xfc, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xe0, 0x00, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0x80, 0x00, 0x03, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xfe, 0x07, 0xff, 0x81, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xfe, 0x1f, 0xff, 0xf0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0x7e, 0x01, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xf0, 0x00, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xc0, 0xfe, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xc7, 0xff, 0xc7, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xf0, 0x7f, 0xff, 0xff, 0xfe, 0x00, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xe0, 0x7f, 0xfe, 0x01, 0xfc, 0x00, 0x03, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xe0, 0x7f, 0xf8, 0x10, 0x3c, 0x00, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xe0, 0x7f, 0xfb, 0xff, 0x3c, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xe0, 0x7f, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xe0, 0x7f, 0xff, 0x83, 0xfc, 0x0f, 0xe0, 0x7f, 0x07, 0x89, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xe0, 0x7f, 0xfc, 0x00, 0xfc, 0x0f, 0xe0, 0x78, 0x00, 0x81, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xe0, 0x7f, 0xf8, 0x00, 0x3c, 0x0f, 0xe0, 0x70, 0x00, 0x51, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xe0, 0x7f, 0xf0, 0x00, 0x3c, 0x0f, 0xe0, 0x70, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xe0, 0x7f, 0xf0, 0x30, 0x1c, 0x0f, 0xc0, 0x70, 0x70, 0x3f, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xe0, 0x7f, 0xe0, 0x7c, 0x1c, 0x00, 0x00, 0x70, 0xf8, 0x3f, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xe0, 0x7f, 0xe0, 0xfc, 0x0c, 0x00, 0x00, 0xff, 0xc0, 0x3f, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xe0, 0x7f, 0xe0, 0xfe, 0x0c, 0x00, 0x01, 0xfc, 0x00, 0x3f, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xe0, 0x7f, 0xe0, 0xfe, 0x0c, 0x00, 0x07, 0xf8, 0x00, 0x3f, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xe0, 0x7f, 0xe0, 0xfe, 0x0c, 0x06, 0x03, 0xf0, 0x00, 0x3f, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xe0, 0x7f, 0xe0, 0xfe, 0x0c, 0x0f, 0x03, 0xe0, 0x70, 0x3f, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xe0, 0x7f, 0xe0, 0xfc, 0x0c, 0x0f, 0x01, 0xe0, 0xf0, 0x3f, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xe0, 0x00, 0x20, 0x7c, 0x1c, 0x0f, 0x81, 0xe0, 0xf0, 0x3f, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xe0, 0x00, 0x30, 0x00, 0x1c, 0x0f, 0x80, 0xe0, 0x00, 0x3f, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xe0, 0x00, 0x30, 0x00, 0x3c, 0x0f, 0xc0, 0xe0, 0x00, 0x3f, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xe0, 0x00, 0x38, 0x00, 0x7c, 0x0f, 0xe0, 0x70, 0x00, 0x3f, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xf0, 0x00, 0x7e, 0x00, 0xfe, 0x0f, 0xe0, 0x78, 0x08, 0x3f, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xf9, 0xff, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xfc, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0x83, 0xff, 0xfc, 0xff, 0xfa, 0x66, 0xef, 0x1e, 0x3b, 0xbb, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xcf, 0xff, 0xef, 0xfc, 0xff, 0xf2, 0x44, 0xce, 0x0c, 0x09, 0x13, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xc3, 0xff, 0x87, 0xfc, 0x26, 0x72, 0x44, 0x8c, 0xc8, 0xc9, 0x17, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xe0, 0x38, 0x0f, 0xfc, 0x26, 0x72, 0x01, 0x0c, 0xf9, 0xf8, 0x07, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xf8, 0x00, 0x3f, 0xf9, 0x94, 0xf2, 0x01, 0xcc, 0xf9, 0x08, 0x8f, 0xff, 
0xff, 0xff, 0xff, 0xfe, 0x7f, 0x83, 0xf9, 0xf9, 0x91, 0xf2, 0x23, 0xcc, 0xc9, 0x18, 0x8f, 0xff, 
0xff, 0xff, 0xff, 0xfe, 0x0f, 0xff, 0xe0, 0xf8, 0x31, 0xe6, 0x23, 0x9c, 0x08, 0x18, 0x9f, 0xff, 
0xff, 0xff, 0xff, 0xff, 0x01, 0xff, 0x01, 0xf8, 0x73, 0xe6, 0x67, 0x9e, 0x3c, 0x39, 0x9f, 0xff, 
0xff, 0xff, 0xff, 0xff, 0x80, 0x00, 0x07, 0xff, 0xe3, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xf0, 0x00, 0x1f, 0xff, 0xc7, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};


char igate_info[52]="";
char meteo_info[52]="";   


void menu();
void righello();
void BottomBanner();
void banner();
void status_display();
void initial_reset();
void load_param();
void make_display();
void make_meteo_display();
//void Xmode_swapp();
void OLED_swapp();

void EEPROM_eraser(byte start, byte stop);
void EEPROM_writer(byte start, byte stop, char tmp_data[50]);
void EEPROM_loader(byte start, byte stop, char tmp_data[50]);
void array_eraser(byte start, byte stop, char tmp_data[50]);
void verifica_parametri();

void APRS_LatLon();
void WiFi_setup();

byte WiFi_setup_retry;
unsigned long WiFi_login_retry;

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
char WiFi_pwd[50]= "";

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

int drift_pres=10;
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

char tmp_buffer[60]="";         // a supporto seriale e menu e caricamento EEPROM
char carMenu;                   // a supporto seriale e menu
char car;                       // a supporto seriale e menu

byte cnt=0;                   // contatore invio stringhe meteo

#define  EEPROM_SIZE  254  // // EEPROM size puo' indirizzare da 0 a 255
#define bottom   ".. (m)enu - (d)isplay - (s)end meteo - (t)ransmit beacon - (r)ead sensor ..\n"
#define DEFAULT_STATUS "https://iw1cgw.wordpress.com/"  

// -------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------



void setup() {
  
 //Init EEPROM
  
  Serial.begin(SERIAL_BAUD);
  Serial.println( "\n" + String(Project) + " v." + String(Release) + "\nmodified by IW1CGW based on OK2DDS' project\n");
  
  //--------------------------------------------------------------------------------------
  display.begin(SSD1306_SWITCHCAPVCC, 0x3c); 
  display.clearDisplay();
  display.drawBitmap(0, 0, logo, 128, 64, WHITE);
  display.display();
  delay(1500);
   //--------------------------------------------------------------------------------------


  EEPROM.begin(EEPROM_SIZE);
  load_param();
  verifica_parametri();
  lora_setup();
  delay(25);

  BM_sensor_status = false;
  BMPstatus = false;
  BMEstatus = false;
  AHTstatus = false;

  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  tmp=9;
   //------------------------------ test del AHT20
  if (!aht.begin()) {}
  else
    {
      Serial.println("AHT20 OK");
      AHTstatus = true;
      display.setCursor(0,tmp);
      display.print("sensor AHT20 OK");
      tmp=tmp+9;
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
          display.setCursor(0,tmp);
          display.print("sensor BME280 0x76 OK");
          tmp=tmp+9;
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
          display.setCursor(0,tmp);
          display.print("sensor BMP280 0x77 OK");
          tmp=tmp+9;
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
          display.setCursor(0,tmp);
          display.print("sensor BMP280 0x76 OK");
          tmp=tmp+9;
        }
    }


  if (BM_sensor_status == false) 
    {
      Serial.println(F("no sensor BMP/BME found"));
      display.setCursor(0,tmp);
      display.print("no BMP/BME found");
      tmp=tmp+9;
    }
 
    display.setCursor(0,tmp);
    display.println("---------------------");
    tmp=tmp+9;
    display.setCursor(0,tmp);
    display.print(Project);
    tmp=tmp+9;
    display.setCursor(0,tmp);
    display.print("version:");
    display.print(Release);
    tmp=tmp+9;
    display.setCursor(0,tmp);
    display.print("build:");
    display.print(Build);
    display.display();
    delay(3000);
  
  // HALL - digital
  if (USE_ANEMOMETER) pinMode(HALL_SENSOR_PIN, INPUT);
  //attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), hall_change, HIGH);

  voltage = float(analogRead(HALL_SENSOR_PIN)) / 4095*2*3.3*1.1;
  delay(10);

  start_server();
  #define DESTCALL_digi    "APLHI0"        
  #define DESTCALL_meteo   "APLHM0"

  Serial.println("Startup finished.\n");
  status_display();
  make_display();
  
  lastMtBeacon = millis() - int(tx_interval * 60000);
  lastIgBeacon = millis() - int(tx_interval * 60000);
  lastStBeacon = millis() - int(tx_interval*8 * 60000);  // lo status del meteo 8 volte il tempo standard della stringa meteo 
  lastStIgBeacon = millis() - int(tx_interval*18 * 60000);// lo status del igate 18 volte il tempo standard della stringa meteo [ ogni 3 ore ] - 
  
  if (oledSwitch == true) display.dim(true);
  else display.dim(false);
  display.display();
  token_verify_update=false;

}

void loop()
{

 
 //---------------------------- OTA   

      if ( check_wifi() && !token_verify_update ) {
        token_verify_update = true;
        if ( checkForUpdates() ) updateFirmware();    
        OTA_logbook(); 
      }

 //---------------------------- OTA




  
  if ( millis() > millis_token_tx +8000 )
    {
      token_tx = HIGH;  // ogni 8 secondi rigenera un token per abilitare il Tx LoRa
      
      if ( backupigateSwitch && !check_wifi() ) digiSwitch = true;
      if ( backupigateSwitch && check_wifi() ) digiSwitch = EEPROM.read( 167 );
      retr++;
      if ( retr>1 ) retr =0;
      if ( retr == 0 ) make_display();
      if ( retr == 1 ) make_meteo_display();
      millis_token_tx = millis();
    }

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
        Tmp="";
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
        lastIgBeacon = millis() - int(tx_interval * 60000);
      }

    if (car == 's' )
      {
        while (Serial.read() != '\n') {};
        lastMtBeacon = millis() - int(tx_interval * 60000);
        lastStBeacon = millis() - int(tx_interval * 60000);
      }

    if (car == 'x' )
      {
        while (Serial.read() != '\n') {};
        checkForUpdates();
        OTA_code = 88;
        OTA_logbook();
        //updateFirmware();
      }


    if (car == 'n' )
      {
        while (Serial.read() != '\n') {};
        cnt_meteo_send = 253;
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

            Serial.print(F("therm:"));
            Serial.print(stemp);
            Serial.println(F(" C."));
                
            Serial.print(F("hum:"));
            Serial.print(sHum);
            Serial.println(F(" %"));

            Serial.print(F("press SLM:"));
            Serial.print(spress);
            Serial.println(F(" hPA"));

            righello();        
          }
      }


    if (car == '#')          
      {
        while (Serial.read() != '\n') {};
        EEPROM_eraser(0,255);
        Serial.println(F("EEPROM erased"));
      }
 
    if (car == 'e')          
      {
        while (Serial.read() != '\n') {};
        tmp = 0;
        while (tmp <= 255)
          {
           Serial.print(tmp); Serial.print(F(" - ")); Serial.println(EEPROM.read(tmp));
           tmp++;    
          } 
      }


  
  if (check_wifi() && wifiStatus == false) 
    {
      Serial.println("WiFi connected");
      wifiStatus = true;
    }

  if (Use_WiFi && !check_wifi() )
   {
    if ( millis() - WiFi_login_retry > 60000 ) WiFi_setup();   // 60 secondi
    }



  if (Use_WiFi && check_wifi()) {
    if (igateSwitch && !check_aprsis() && aprsLastReconnect + 60000 < millis() || meteo_tx_mode >1  && !check_aprsis() && aprsLastReconnect + 60000 < millis() ) {
      aprsis.stop();
      delay(100);
      aprsLastReconnect = millis();
      aprsis_connect();
    }
    if (myIP != ipToString(WiFi.localIP())) {
      myIP = ipToString(WiFi.localIP());
      Serial.println("IP:" + myIP);
      make_display();
    }
    

    WiFiClient client = server.available();
  

    if (client) {                             
    unsigned long currentTime = millis();
    unsigned long previousTime = currentTime;
    String header;
    if (HTTP_DEBUG_MODE)  Serial.println("HTTP from " + ipToString(client.remoteIP()) + ":" + String(client.remotePort()));     
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
              //client.println("{\"general\": {\"version\":\"" + String(Release) + "\", \"destcall\":\"" + String(DESTCALL) + "\", \"system_time\":" + String(millis()) + ", \"voltage\":" + String(voltage) + ", \"battery\":" + String(battPercent) + ", \"wifi_status\":" + (check_wifi() ? "true" : "false") + ", \"wifi_signal_db\":" + (check_wifi() ? String(WiFi.RSSI()) : "0") + ", \"wifi_ssid\":\"" + String(WiFi.SSID()) + "\", \"wifi_hostname\":\"" + String(Hostname) + "\", \"bmp280_status\":" + (getBM_sensor_status() ? "true" : "false") + "}, \"lora\": {\"METEO_CALLSIGN\":\"" + String(METEO_CALLSIGN) + "\", \"digi_enabled\":" + (digiSwitch ? "true" : "false") + "\", \"meteo_enabled\":" + (meteoSwitch ? "true" : "false") + "\", \"aprs_is_enabled\":" + (meteoAPRSswitch ? "true" : "false") + ", \"igate_callsign\":\"" + String(IGATE_CALLSIGN) + "\", \"aprs_is_enabled\":" + (aprsSwitch ? "true" : "false") + ", \"aprs_is_status\":" + (check_aprsis() ? "true" : "false") + ", \"aprs_server\":\"" + (check_aprsis() ? String(APRSISServer) : "disconnected") + "\", \"hall_sensor\":" + String(anemoACValue) + ", \"last_rx\":\"" + String(lastRXstation) + "\"" + "}, \"meteo\": {\"temperature\":" + valueForJSON(tempToWeb(getTempC())) + ", \"pressure\":" + valueForJSON(pressToWeb(getPressure()))  + ", \"Hum\":" + valueForJSON(HumToWeb(getHum())) + ", \"actual_wind\":" + valueForJSON(windToWeb(windActualSpeed)) + ", \"long_period_wind\":" + valueForJSON(windToWeb(windLongPeriodSpeed)) + ", \"gust\":" + valueForJSON(windToWeb(gust)) + ", \"min_temperature\":" + (getBM_sensor_status() ? String(minTemp) : "0") + ", \"max_temperature\":" + (getBM_sensor_status() ? String(maxTemp) : "0") + ", \"min_pressure\":" + (getBM_sensor_status() ? String(minPress) : "0") + ", \"max_pressure\":" + (getBM_sensor_status() ? String(maxPress) : "0") + ", \"max_wind\":" + String(maxWind) + ", \"max_gust\":" + String(maxGust) + "}}");
              //  client.println("{\"general\": {\"version\":\"" + String(Release) + "\", \"destcall\":\"" + String(DESTCALL) + "\", \"system_time\":" + String(millis()) + ", \"voltage\":" + String(voltage) + ", \"battery\":" + String(battPercent) + ", \"wifi_status\":" + (check_wifi() ? "true" : "false") + ", \"wifi_signal_db\":" + (check_wifi() ? String(WiFi.RSSI()) : "0") + ", \"wifi_ssid\":\"" + String(WiFi.SSID()) + "\", \"wifi_hostname\":\"" + String(Hostname) + "\", \"bmp280_status\":" + (getBM_sensor_status() ? "true" : "false") + "}, \"lora\": {\"METEO_CALLSIGN\":\"" + String(METEO_CALLSIGN) + "\", \"digi_enabled\":" + (digiSwitch ? "true" : "false") + "\", \"meteo_enabled\":" + (meteoSwitch ? "true" : "false") + "\", \"send_data_meteo\":" + (meteoAPRSswitch ? "true" : "false") + ", \"igate_callsign\":\"" + String(IGATE_CALLSIGN) + ", \"aprs_is_status\":" + (check_aprsis() ? "true" : "false") + ", \"aprs_server\":\"" + (check_aprsis() ? String(APRSISServer) : "disconnected") + "\", \"hall_sensor\":" + String(anemoACValue) + ", \"last_rx\":\"" + String(lastRXstation) + "\"" + "}, \"meteo\": {\"temperature\":" + valueForJSON(tempToWeb(getTempC())) + ", \"pressure\":" + valueForJSON(pressToWeb(getPressure()))  + ", \"Hum\":" + valueForJSON(HumToWeb(getHum())) + ", \"actual_wind\":" + valueForJSON(windToWeb(windActualSpeed)) + ", \"long_period_wind\":" + valueForJSON(windToWeb(windLongPeriodSpeed)) + ", \"gust\":" + valueForJSON(windToWeb(gust)) + ", \"min_temperature\":" + (getBM_sensor_status() ? String(minTemp) : "0") + ", \"max_temperature\":" + (getBM_sensor_status() ? String(maxTemp) : "0") + ", \"min_pressure\":" + (getBM_sensor_status() ? String(minPress) : "0") + ", \"max_pressure\":" + (getBM_sensor_status() ? String(maxPress) : "0") + ", \"max_wind\":" + String(maxWind) + ", \"max_gust\":" + String(maxGust) + "}}");
              //client.println("{\"general\": {\"version\":\"" + String(Release) + "\", \"destcall\":\"" + String(DESTCALL) + "\", \"system_time\":" + String(millis()) + ", \"voltage\":" + String(voltage) + ", \"battery\":" + String(battPercent) + ", \"wifi_status\":" + (check_wifi() ? "true" : "false") + ", \"wifi_signal_db\":" + (check_wifi() ? String(WiFi.RSSI()) : "0") + ", \"wifi_ssid\":\"" + String(WiFi.SSID()) + "\", \"wifi_hostname\":\"" + String(Hostname) + "\", \"bmp280_status\":" + (getBM_sensor_status() ? "true" : "false") + "}, \"lora\": {\"METEO_CALLSIGN\":\"" + String(METEO_CALLSIGN) + "\", \"digi_enabled\":" + (digiSwitch ? "true" : "false") + "\", \"igate_callsign\":\"" + String(IGATE_CALLSIGN) + ", \"aprs_is_status\":" + (check_aprsis() ? "true" : "false") + ", \"aprs_server\":\"" + (check_aprsis() ? String(APRSISServer) : "disconnected") + "\", \"hall_sensor\":" + String(anemoACValue) + ", \"last_rx\":\"" + String(lastRXstation) + "\"" + "}, \"meteo\": {\"temperature\":" + valueForJSON(tempToWeb(getTempC())) + ", \"pressure\":" + valueForJSON(pressToWeb(getPressure()))  + ", \"Hum\":" + valueForJSON(HumToWeb(getHum())) + ", \"actual_wind\":" + valueForJSON(windToWeb(windActualSpeed)) + ", \"long_period_wind\":" + valueForJSON(windToWeb(windLongPeriodSpeed)) + ", \"gust\":" + valueForJSON(windToWeb(gust)) + ", \"min_temperature\":" + (getBM_sensor_status() ? String(minTemp) : "0") + ", \"max_temperature\":" + (getBM_sensor_status() ? String(maxTemp) : "0") + ", \"min_pressure\":" + (getBM_sensor_status() ? String(minPress) : "0") + ", \"max_pressure\":" + (getBM_sensor_status() ? String(maxPress) : "0") + ", \"max_wind\":" + String(maxWind) + ", \"max_gust\":" + String(maxGust) + "}}");
              client.println("{\"general\": {\"version\":\"" + String(Release) + "\", \"system_time\":" + String(millis()) + ", \"voltage\":" + String(voltage) + ", \"battery\":" + String(battPercent) + ", \"wifi_status\":" + (check_wifi() ? "true" : "false") + ", \"wifi_signal_db\":" + (check_wifi() ? String(WiFi.RSSI()) : "0") + ", \"wifi_ssid\":\"" + String(WiFi.SSID()) + "\", \"wifi_hostname\":\"" + String(Hostname) + "\", \"bmp280_status\":" + (getBM_sensor_status() ? "true" : "false") + "}, \"lora\": {\"METEO_CALLSIGN\":\"" + String(METEO_CALLSIGN) + "\", \"digi_enabled\":" + (digiSwitch ? "true" : "false") + "\", \"igate_callsign\":\"" + String(IGATE_CALLSIGN) + ", \"aprs_is_status\":" + (check_aprsis() ? "true" : "false") + ", \"aprs_server\":\"" + (check_aprsis() ? String(APRSISServer) : "disconnected") + "\", \"hall_sensor\":" + String(anemoACValue) + ", \"last_rx\":\"" + String(lastRXstation) + "\"" + "}, \"meteo\": {\"temperature\":" + valueForJSON(tempToWeb(getTempC())) + ", \"pressure\":" + valueForJSON(pressToWeb(getPressure()))  + ", \"Hum\":" + valueForJSON(HumToWeb(getHum())) + ", \"actual_wind\":" + valueForJSON(windToWeb(windActualSpeed)) + ", \"long_period_wind\":" + valueForJSON(windToWeb(windLongPeriodSpeed)) + ", \"gust\":" + valueForJSON(windToWeb(gust)) + ", \"min_temperature\":" + (getBM_sensor_status() ? String(minTemp) : "0") + ", \"max_temperature\":" + (getBM_sensor_status() ? String(maxTemp) : "0") + ", \"min_pressure\":" + (getBM_sensor_status() ? String(minPress) : "0") + ", \"max_pressure\":" + (getBM_sensor_status() ? String(maxPress) : "0") + ", \"max_wind\":" + String(maxWind) + ", \"max_gust\":" + String(maxGust) + "}}");



            } else {
            client.println(String(webPageStart));
            
            if (!GETIndex(header, "/watch")) client.println(String(webPageHeader) + "</h1><br>");
            
            if (GETIndex(header, "/switch-meteo")) {
              meteo_tx_mode++;
              if ( meteo_tx_mode> 3 ) meteo_tx_mode=0;
              EEPROM.write(164,meteo_tx_mode);
              EEPROM.commit();
              client.println(webReload);
            }
            
            if (GETIndex(header, "/switch-aprs")) {
              igateSwitch = !igateSwitch;
              verifica_parametri();
              if (igateSwitch ) EEPROM.write(166,1);
              else EEPROM.write(166,0);
              EEPROM.commit();
              client.println(webReload);
              aprsis.stop();
              delay(100);
            }
          
            if (GETIndex(header, "/switch-backup_igate")) {
              backupigateSwitch = !backupigateSwitch;
              if (backupigateSwitch == true ) EEPROM.write(165,1);
              if (backupigateSwitch == false) EEPROM.write(165,0);
              EEPROM.commit();
              make_display();
              client.println(webReload);
              delay(100);
            }

            if (GETIndex(header, "/switch-digi")) {
              digiSwitch = !digiSwitch;
              verifica_parametri();
              if (digiSwitch == true ) EEPROM.write(167,1);
              if (digiSwitch == false) EEPROM.write(167,0);
              EEPROM.commit();
              client.println(webReload);
              delay(100);
            }

            if (GETIndex(header, "/update")) {
              client.println("<br><a>.. check for update ..</a>");
              if ( checkForUpdates() ) {
                client.println("<br><a>.. an update is available ..</a>");
                client.println("<br><a>.. installing the update ..</a>");
                client.println("<br><a>.. please, close this tab ..</a>");
                updateFirmware();
              } 
              else {
                client.println("<br><a>.. no update is available ..</a>");
                OTA_code = 99;
                OTA_logbook(); 
                delay(1000);
                client.println(webReload);
              }
            }
           
           if (GETIndex(header, "/restart")) {
              client.println("<br><a>.. restart in progress ..</a>");
              delay(1000);
              client.println("<br><a>.. please, close this tab ..</a>");
              delay(1000);
              ESP.restart();
            }

            
            if (GETIndex(header, "/beacon")) {
              if ( meteo_tx_mode >0 || digiSwitch || igateSwitch ){
                lastIgBeacon = millis() - int(tx_interval * 60000);
                lastMtBeacon = millis() - int(tx_interval * 60000);
                client.println("<br>.. sending beacons ..<br>");
              }
              else client.println("<br>.. all systems are off, no beacons sent ..<br>");
              delay(2000);
              client.println(webReload);
            }
           

            if (GETIndex(header, "/reset-temp")) {
              minTemp = getTempC();
              maxTemp = minTemp;
              client.println("<br>Temperature reset done.<br>");
              delay(1000);
              client.println(webReload);
            }
            if (GETIndex(header, "/reset-hum")) {
              minHum = getHum();
              maxHum = minHum;
              client.println("<br>Humidity reset done.<br>");
              delay(1000);
              client.println(webReload);
            }

            if (GETIndex(header, "/reset-press")) {
              minPress = getPressure();
              maxPress = minPress;
              client.println("<br>Pressure reset done.<br>");
              delay(1000);
              client.println(webReload);
            }



            if (GETIndex(header, "/lora"))
               // -- original // client.println("<br>Version: " + String(VERSION) + "<br><br> Voltage: " + String(voltage) + "V<br>Battery: " + String(battPercent) + "%<br>Wi-Fi: " + (check_wifi() ? String(WiFi.SSID()) + " " + String(WiFi.RSSI()) + " dB<br>IP: " + ipToString(WiFi.localIP()) : String("not connected")) + String("<br>APRS-IS: ") + (aprsis.connected() ? "connected" : "not connected") + "<br>Last RX: " + String(lastRXstation) + "<br>Hall sensor: " + String(anemoACValue) + "<br><br>");
               client.println("Version: " + String(Release) + " build: " + String(Build) + "<br><br> Voltage: " + String(voltage) + " - Battery: " + String(battPercent) + "%<br>Wi-Fi: " + (check_wifi() ? String(WiFi.SSID()) + " " + String(WiFi.RSSI()) + " dBm<br>IP: " + ipToString(WiFi.localIP()) : String("not connected")) + String("<br>APRS-IS: ") + (aprsis.connected() ? "connected" : "not connected") + "<br>iGate/digi: " + IGATE_CALLSIGN + " - meteo: " + METEO_CALLSIGN +"<br>Last RX: " + String(lastRXstation) + " SNR=" + String(LoRa.packetSnr()) + " dB RSSI=" + String(LoRa.packetRssi()) + " dBm" + "<br><br>");
            if ((GETIndex(header, "/lora") || GETIndex(header, "/min-max")) && meteo_tx_mode >0  && (BM_sensor_status || USE_ANEMOMETER)) {
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
              else client.println("<br>reset: <a href='/reset-temp'>Temperature</a> - <a href='/reset-press'>Pressure</a><br>");
        
              client.println("drift: ");
              client.println("<a href='/change-drift_temp'>Temperature</a>:");
              client.println(" " + String(drift_thermC) + " - ");
              client.println("<a href='/change-drift_pres'>Pressure</a>:");
              client.println(" " + String(drift_pres));
               
              if (meteo_tx_mode == 0 ) Tmp = "disabled";
              if (meteo_tx_mode == 1 ) Tmp  = "to RF";
              if (meteo_tx_mode == 2 ) Tmp  = "to ip";
              if (meteo_tx_mode == 3 ) Tmp  = "to RF + ip";
              make_display();
              client.println("<br><br>meteo send <a href='/switch-meteo'> is</a>: " + Tmp);
                      
              
              client.println("<br>iGate <a href='/switch-aprs'>On/Off</a>: (" + String(igateSwitch ? "ON" : "OFF") + ")");
              client.println("<br>digipeater <a href='/switch-digi'>On/Off</a>: (" + String(digiSwitch ? "ON" : "OFF") + ")");
              client.println("<br>backup iGate/digi <a href='/switch-backup_igate'>On/Off</a>: (" + String(backupigateSwitch ? "ON" : "OFF") + ")");
         
              client.println("<br>power <a href='/power'>is</a>: " + String(LoRa_power) + " dBm - send beacon <a href='/beacon'>now</a>");
              client.println("<br><a href='/restart'>reboot</a> - check for <a href='/update'>update</a>");
              
              client.println("<br><br>-------------------------------------------");
              client.println("<br>change <a href='/change-igate_info'>igate info</a>: " + String(igate_info));
              client.println("<br>change <a href='/change-meteo_info'>meteo info</a>: " + String(meteo_info));
              client.println("<br>-------------------------------------------");
              

   
              // ------------ dash
                    
            }

            if (GETIndex(header, "/power")) {
              Tmp="power";
              client.println(web_ChangePrompt_power);
            }

            if (GETIndex(header, "/change-drift_temp")) {
              Tmp="drift_temp";
              client.println(web_ChangePrompt_temp);
            }

            if (GETIndex(header, "/change-drift_pres")) {
              Tmp="drift_pres";
              client.println(web_ChangePrompt_pres);
            }

            if (GETIndex(header, "/change-meteo_info")) {
              Tmp="meteo_info";
              client.println(web_ChangePrompt_meteo);
            }

            if (GETIndex(header, "/change-igate_info")) {
              Tmp="igate_info";
              client.println(web_ChangePrompt_igate);
            }

            if (GETIndex(header, "/new-value")) {
              apktIndex = header.indexOf("GET /new-value");
              String newData = header.substring(apktIndex + 15, header.indexOf("HTTP/") - 1);
              if (newData != "null" ) {
                newData.replace("%20", " ");  // rimpiazza eventuali %20 con spazi
                ptr = newData.length();       // lunghezza della stringa
                if ( ptr > 50 ) ptr = 50;
                
                if (Tmp == "igate_info" ) {
                    newData.toCharArray(igate_info,52);
                    EEPROM_writer(112,112+ptr-1,igate_info);
                    EEPROM_eraser(112+ptr,163);
                }    
                if (Tmp == "meteo_info" ) {
                  newData.toCharArray(meteo_info,52);
                  EEPROM_writer(60,60+ptr-1,meteo_info);
                  EEPROM_eraser(60+ptr,111);
                }   
                if (Tmp == "drift_temp" ) {
                  if (ptr > 5 ) ptr = 5;
                  char tmp_drift_thermC[6];
                  newData.toCharArray(tmp_drift_thermC,6);   
                  if (atof(tmp_drift_thermC) <= 5 && atof(tmp_drift_thermC) >= -5 ) {
                   newData.toCharArray(drift_thermC,6);
                   EEPROM_writer(47,47+ptr-1,drift_thermC);                   // salvare nella eeprom il varore in Char
                   EEPROM_eraser(47+ptr,51);
                   drift_therm = atof(drift_thermC);                          // valore matematico
                  }
                }   
                if (Tmp == "drift_pres" ) {
                  if (ptr > 5 ) ptr = 5;
                  newData.toCharArray(tmp_buffer,60);   
                   if (atoi(tmp_buffer) >=-10 && atoi(tmp_buffer) <= 10 ) {
                    drift_pres = atoi(tmp_buffer);                            // valore matematico
                    EEPROM.write(169, (drift_pres+10));
                    EEPROM.commit();                          
                  }   
                }  
                if (Tmp == "power" ) {
                  if (ptr > 2 ) ptr = 2;
                  newData.toCharArray(tmp_buffer,60);   
                   if (atoi(tmp_buffer) <= 20 ) {
                    LoRa_power = atoi(tmp_buffer);         
                    LoRa.setTxPower(LoRa_power);
                    delay(1000);
                    EEPROM.write(39, (LoRa_power));
                    EEPROM.commit();                          
                  }  
                }   
               } else {
                client.println(web_ChangeError);
                delay(1000);
              }
              
              apktIndex = 0;
              client.println(webReload);
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
              client.println("<a href='/'>view main meteo page</a>");
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
            



            if (GETIndex(header, "/ ")) {
              // ORDINARY METEO WEBSITE
              client.println(webMeteoOnlineIndicator);
              client.println(webMeteoLayout);
              client.println(webSocketSetupScript);
              client.println(HTMLelementDef("onlineIndicator") + HTMLelementDef("temp") + HTMLelementDef("Hum") + HTMLelementDef("press") + HTMLelementDef("wind") + HTMLelementDef("windkmh") + HTMLelementDef("gust") + HTMLelementDef("windlp"));
              
              client.println(webMeteoOnlineRoutine);
              if ( IGATE_CALLSIGN.substring(0, 1) != "R" ) client.println(webSocketHandleScript);  
              if ( IGATE_CALLSIGN.substring(0, 1) == "R" ) client.println(webSocketHandleScript_RU);//--- dash RU
            }
           
            
            if (GETIndex(header, "/watch")) {
              // METEO WEBSITE LAYOUT FOR WATCH
              client.println(webMeteoWatchLayout);
              client.println("<script>var dat = '" + tempToWeb(getTempC()) + "," + HumToWeb(int(getHum())) + pressToWeb(getPressure()) + "," + windToWeb(windActualSpeed) + "," + windToWeb(windKMH(windActualSpeed)) + "'; ");
              client.println(HTMLelementDef("temp") + HTMLelementDef("Hum") +  HTMLelementDef("press") + HTMLelementDef("wind") + HTMLelementDef("windkmh"));
                          
              if (IGATE_CALLSIGN.substring(0, 1) != "R" ) client.println(webWatchValuesScript);
              if (IGATE_CALLSIGN.substring(0, 1) == "R" ) client.println(webWatchValuesScript_RU);//--- dash RU
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
 
  //if (aprsSwitch && check_wifi() && check_aprsis() && lastIgBeacon + (tx_interval * 60000) < millis()) beacon_igate();
  if (lastIgBeacon + (tx_interval * 60000) < millis()) beacon_igate();
  if (lastStIgBeacon + (tx_interval*18 * 60000) < millis()) beacon_igate_status();  // lo status ogni 3 ore - se iGate riceve segnali resetta pure il contatore
  if (meteo_tx_mode >0)
    {
      if (lastMtBeacon + (tx_interval * 60000) < millis()) beacon_meteo();
      if (lastStBeacon + (tx_interval*8 * 60000) < millis()) beacon_meteo_status();
    }   
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
    

    rxPacket = rxPacket.substring(3);
    Serial.println("RX: " + rxPacket.substring(0,(rxPacket.length()-1)));

    if (!(rxPacket.length() < 5 || rxPacket.indexOf('>') < 5 || rxPacket.indexOf(':') < rxPacket.indexOf('>') || rxPacket.substring(rxPacket.indexOf('>') + 1, rxPacket.indexOf(':')) == "") && igateSwitch && Use_WiFi && igateSwitch) {
      String igatePacket = rxPacket;
    //if (igatePacket.indexOf("NOGATE") == -1 && igatePacket.indexOf("RFONLY") == -1 && igatePacket.indexOf("TCPIP") == -1 && igatePacket.indexOf("TCPXX") == -1 && igatePacket.indexOf(String(METEO_CALLSIGN)) == -1 && igatePacket.indexOf(String(IGATE_CALLSIGN) + "*") == -1 && rxPacket.substring(0, rxPacket.indexOf('>')) != String(IGATE_CALLSIGN)) {
      if (igatePacket.indexOf("NOGATE") == -1 && igatePacket.indexOf("RFONLY") == -1 && igatePacket.indexOf("TCPIP") == -1 && igatePacket.indexOf("TCPXX") == -1                                                                                                                 && rxPacket.substring(0, rxPacket.indexOf('>')) != String(IGATE_CALLSIGN)) {
        igatePacket = igatePacket.substring(0, igatePacket.indexOf(":")) + ",qAO," + String(IGATE_CALLSIGN) + igatePacket.substring(igatePacket.indexOf(":"));
        aprsis_send(igatePacket);
      }
    }

    // digipeating
    // digipeating
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
      
      // dash
      //lastRXstation = sourceCall;
      if (originalPath.indexOf("*") == -1) lastRXstation = sourceCall;
      if (originalPath.indexOf("*") > 0 ) lastRXstation = rxPacket.substring(rxPacket.indexOf(',') + 1, rxPacket.indexOf('*'));

      if (int callIndex = digiPath.indexOf(String(IGATE_CALLSIGN)) > -1 && digiPath.indexOf(String(IGATE_CALLSIGN) + "*") == -1) {
        digiPath = digiPath.substring(0, callIndex - 1) + digiPath.substring(callIndex + String(IGATE_CALLSIGN).length());
      }
      if   (int paradigmIndex = digiPath.indexOf("WIDE1-") > -1 && digiSwitch && digiPath.indexOf(String(IGATE_CALLSIGN) + "*") == -1 && rxPacket.indexOf(String(METEO_CALLSIGN)) == -1 && sourceCall.indexOf(String(IGATE_CALLSIGN)) == -1) {
      //if (int paradigmIndex = digiPath.indexOf("WIDE1-") > -1 && digiSwitch                                                                                                                                                              ) {        
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
        delay(300);
        lora_send(digiPacket);
      }   else if (digiSwitch && DIGI_IGNORE_PARADIGM && digiPath.indexOf("*") == -1 && (millis() > lastDigipeat + 600000 || lastDigipeat == 0) && digiPath.indexOf(String(IGATE_CALLSIGN) + "*") == -1 && rxPacket.indexOf(String(METEO_CALLSIGN)) == -1 && sourceCall.indexOf(String(IGATE_CALLSIGN)) == -1) {
        //else if (digiSwitch && DIGI_IGNORE_PARADIGM && digiPath.indexOf("*") == -1 && (millis() > lastDigipeat + 600000 || lastDigipeat == 0)                                                                                                                                                              ) {
        
        lastDigipeat = millis();
        digiPath = digiPath + "," + String(IGATE_CALLSIGN) + "*";
        if (digiPath.indexOf(",") != 0)
          digiPath = "," + digiPath;
        // do not add SNR and RSSI
        digiPacket = sourceCall + ">" + destCall + digiPath + ":" + message;
        delay(300);
        (digiPacket);
	    // do not digipeat without WIDE1-1
      } else if (digiSwitch && DIGI_IGNORE_PARADIGM) {
        Serial.println("Station not repeated.");
      }
      digiOutput = true;

      // dash
      // send status
      //statusMessage = String(IGATE_CALLSIGN) + ">APLHI0:>Last RX: " + String(sourceCall) + " SNR=" + String(LoRa.packetSnr()) + "dB RSSI=" + String(LoRa.packetRssi()) + "dBm";
      statusMessage = String(IGATE_CALLSIGN) + ">APLHI0:>Last RX: " + String(lastRXstation) + " SNR=" + String(LoRa.packetSnr()) + "dB RSSI=" + String(LoRa.packetRssi()) + "dBm";
      //if (igateSwitch && USE_LASTRX_STATUS && originalPath.indexOf("*") == -1)
      if (igateSwitch && USE_LASTRX_STATUS && lastRXstation != "no station")
        aprsis_send(statusMessage);
        lastStIgBeacon=millis();    // per 3 ore non verrà inviato lo status 'istituzionale' cosi rimane l'ultima stazione ricevuta.
      bad_packet:
      if (!digiOutput) Serial.println("Bad packet");
  }
  }

/*

  while (igateSwitch && check_aprsis() && aprsis.available()) { 
    String apstring;
    char aprx = aprsis.read();
    apstring += aprx;
    if (aprx == '\n') {
      
      //------------------------------------
      //Serial.println(apstring);
      //------------------------------------

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


*/


} //--- end of loop


// ------------------------------------------------------------------
// ------------------------------------------------------------------
// ------------------------------------------------------------------



void lora_setup() {
  SPI.begin(LoRa_SCK, LoRa_MISO, LoRa_MOSI, LoRa_SS);
  LoRa.setPins(LoRa_SS, LoRa_RST, LoRa_DIO0);
  if (!LoRa.begin(atoi(frequencyC)*1000)) {
    Serial.println("Failed to setup LoRa module.");
    while (1);
  }
/*
  if ( Xmode )
   {
      LoRa.setSpreadingFactor(LoRa_XSpreadingFactor);   // --- parametri veloci
      LoRa.setSignalBandwidth(LoRa_XSignalBandwidth);
      LoRa.setCodingRate4(LoRa_XCodingRate4);
   }  
  else
*/
   {  
      LoRa.setSpreadingFactor(LoRa_SpreadingFactor);    // --- parametri std
      LoRa.setSignalBandwidth(LoRa_SignalBandwidth);
      LoRa.setCodingRate4(LoRa_CodingRate4);
    }
  
  LoRa.enableCrc();
  LoRa.setTxPower(LoRa_power);
  delay(1000);

 /* 
  if (!igateSwitch && !digiSwitch && meteo_tx_mode==0) {
    LoRa.sleep();
    Serial.println(F(""));
    Serial.println(F("*** LoRa module set to sleep mode ! ***"));
  }
 */
}








void lora_send(String tx_data)
 {
  LoRa.setFrequency(atoi(frequencyC)*1000);
  LoRa.beginPacket();
  LoRa.write('<');
  LoRa.write(0xFF);
  LoRa.write(0x01);
  //Serial.println("TX: " + tx_data);
  Serial.println("TX: " + tx_data.substring(0,(tx_data.length()-1)));
  
  token_tx = false;
  millis_token_tx = millis();
  
  LoRa.write((const uint8_t *)tx_data.c_str(), tx_data.length());
  LoRa.endPacket();
  
  //LoRa.setFrequency(atoi(frequencyC)*1000);
  /*
  if (!igateSwitch) {
    LoRa.sleep();
  }
  */
}


void aprsis_connect() {
  if (Use_WiFi && check_wifi() && igateSwitch || Use_WiFi && check_wifi() && meteo_tx_mode > 1 ) 
  {
    if (aprsis.connect(APRSISServer.c_str(), APRS_IS_Port)) {
      Serial.println("server APRS-IS connected");
    } else {
      Serial.println("server APRS-IS error");
      return;
    }
    
    if ( igateSwitch ) aprsis.println("user " + String(IGATE_CALLSIGN) + " pass " + String(aprs_passcode) + " vers IW1CGW_LoRa_iGate-Wx " + String(Release));
    else if ( meteo_tx_mode >1 ) aprsis.println("user " + String(METEO_CALLSIGN) + " pass " + String(aprs_passcode) + " vers IW1CGW_LoRa_iGate-Wx " + String(Release));

    delay(3000);
    //String Tmp;
    retr=20;
    while (aprsis.available() || retr == 0 )
      {
        aprsis.readStringUntil('\r');
        Tmp=(aprsis.readString());
        Serial.println(Tmp);
        delay(350);
        --retr;
       }
    retr = Tmp.indexOf("unverified");
    //Serial.print("valore di retr: "); Serial.println(retr);
    if ( retr == 255 ) 
      {
        APRS_login = true;
        Serial.println(F("server APRS-IS login successfull"));
      }  
    else
      {
        APRS_login = false;
        Serial.println(F("server APRS-IS login failure, check your passocode"));
      }
    }
}





void aprsis_send(String aprsis_packet)
 {
    if (!check_aprsis() )
      {
        aprsis.stop();
        delay(100);
        aprsis_connect();
      } 
    else
      {
        if (check_aprsis())
          {
            Serial.println("APRS-IS: " + String(aprsis_packet));
            aprsis.println(aprsis_packet);
          }
        else
          {
            Serial.println("APRS-IS TX error");
          }
      }
 }








void beacon_meteo()
 {
  if (token_tx == HIGH )
   {
    
    lastMtBeacon=millis();  

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
  
  if (meteo_tx_mode >0 )
    {
     String meteoBeacon = String(METEO_CALLSIGN) + ">" + String(DESTCALL_meteo)              + ":!" + String(lat_meteo_APRS) + "/" + String(lon_meteo_APRS) + "_.../" + String(windSpeedAPRS(windLongPeriodSpeed)) + "g" +  String(windSpeedAPRS(gust)) + "t" + String(getTempAPRS()) + "r...p...P..." + "h" + String(getHumAPRS()) + "b" + String(getPressureAPRS()) + " " + String(meteo_info);
     if ( meteo_tx_mode > 1 && check_wifi() ) aprsis_send(meteoBeacon);  // se iGate acceso e connesso manda in APRS-IS
            meteoBeacon = String(METEO_CALLSIGN) + ">" + String(DESTCALL_meteo) + ",WIDE1-1" + ":!" + String(lat_meteo_APRS) + "/" + String(lon_meteo_APRS) + "_.../" + String(windSpeedAPRS(windLongPeriodSpeed)) + "g" +  String(windSpeedAPRS(gust)) + "t" + String(getTempAPRS()) + "r...p...P..." + "h" + String(getHumAPRS()) + "b" + String(getPressureAPRS()) + " " + String(meteo_info) + " [" + String(cnt_meteo_send) + "]" + char(10);
     if (meteo_tx_mode == 1 || meteo_tx_mode == 3 ) lora_send(meteoBeacon);    // se digipeater attivo trasmetti in LoRa
     if (cnt_meteo_send == 255) cnt_meteo_send = 0;
     else cnt_meteo_send++;
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
}


void beacon_meteo_status()
 {
  if (token_tx == HIGH)
  {
      lastStBeacon=millis();
      String meteoStatus = String(METEO_CALLSIGN) + ">" + String(DESTCALL_meteo) + ":>" + String(DEFAULT_STATUS);
      if ( meteo_tx_mode > 1 && check_wifi() ) aprsis_send(meteoStatus); // se wifi ok e igate acceso e meteo_tx_mode 2 o 3 manda status in APRS-IS
      
             meteoStatus = String(METEO_CALLSIGN) + ">" + String(DESTCALL_meteo) + ",WIDE1-1" + ":>" + String(DEFAULT_STATUS) + char(10);
      if (meteo_tx_mode == 1 || meteo_tx_mode == 3 ) lora_send(meteoStatus);  // 
   } 
}

//-- dash
void beacon_igate()
 {
  if (token_tx == HIGH )
    {
    lastIgBeacon = millis();
    String icon = "#";    // di base l'icona é quella del digipeater la 'L' nella stella verde
    if ( check_wifi() && check_aprsis() && igateSwitch ) icon = "&";   // se igate attivo a tutti gli effetti con WiFi ok e APRS-IS in tiro icona L in rombo nero
    
    String beacon = String(IGATE_CALLSIGN) + ">" + String(DESTCALL_digi)              + ":!" + String(lat_igate_APRS) + "L" + String(lon_igate_APRS) + icon + String(igate_info) + String(" | batt:") + String(voltage)+"V";
    if (igateSwitch && check_wifi() && check_aprsis() ) aprsis_send(beacon);
         beacon = String(IGATE_CALLSIGN) + ">" + String(DESTCALL_digi) + ",WIDE1-1" + ":!" + String(lat_igate_APRS) + "L" + String(lon_igate_APRS) + icon + String(igate_info) + String(" | batt:") + String(voltage)+"V" + char(10);
    if (digiSwitch && token_tx) lora_send(beacon);
    }
 }

void beacon_igate_status()
 {
  if (token_tx == HIGH)
    {
      lastStIgBeacon=millis();  
      String IgStatus = String(IGATE_CALLSIGN) + ">" + String(DESTCALL_digi) + ":>" + String(DEFAULT_STATUS);
      if (igateSwitch && check_wifi() ) aprsis_send(IgStatus); // se wifi ok e igate acceso manda status in APRS-IS
             IgStatus = String(IGATE_CALLSIGN) + ">" + String(DESTCALL_digi) + ",WIDE1-1" + ":>" + String(DEFAULT_STATUS) + char(10);
      if (digiSwitch && token_tx ) lora_send(IgStatus);  // 
  } 
}



bool check_wifi() {
  if (WiFi.status() == WL_CONNECTED)
    return true;
  else
    return false;
}



bool check_aprsis() {
  if (aprsis.connected() && check_wifi() && APRS_login )
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
      
        if(BMPstatus && atoi(altitude) <= 400 ) return drift_pres+( (bmp.readPressure()           * ( pow(1.0 - (0.0065 * atoi(altitude) * -1 / (273.15 + getTempC() )), 5.255)) ) / 100 );
        if(BMPstatus && atoi(altitude) >  400 ) return drift_pres+( bmp.readPressure() / pow((1-((float)(atoi(altitude)))/44330), 5.255))/100.0; 

        if(BMEstatus && atoi(altitude) <= 400 )
          {
            bme_pressure->getEvent(&pressure_event);
            return drift_pres + ( ((pressure_event.pressure*100) * ( pow(1.0 - (0.0065 * atoi(altitude) * -1 / (273.15 + getTempC() )), 5.255)) ) / 100 );
          }
        if(BMEstatus && atoi(altitude) > 400 )
          {
            bme_pressure->getEvent(&pressure_event);
            return drift_pres + ( (pressure_event.pressure *100) /pow((1-((float)(atoi(altitude)))/44330), 5.255))/100.0; 
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
  
  int ifahrenheit_abs = abs(ifahrenheit);
  String sfahrenheit = String(ifahrenheit_abs);
  if ( ifahrenheit_abs < 100 && ifahrenheit >= 0 ) sfahrenheit = String("0") + sfahrenheit;
  if ( ifahrenheit_abs < 10                      ) sfahrenheit = String("0") + sfahrenheit;
  if ( ifahrenheit < 0 ) sfahrenheit = String("-") + sfahrenheit;
  return sfahrenheit;
  } else return "...";
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
    return ".....";
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
    if ( HTTP_DEBUG_MODE) Serial.println("WS Con: " + String(ws.count()) + " connected.");
    isWSconnected = true;
  } else if(type == WS_EVT_DISCONNECT){
    if ( HTTP_DEBUG_MODE) Serial.println("WS Dis: " + String(ws.count()) + " connected.");
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
  // dash RU
  //pressValue = 1020.0;    // only for test 1020.0 hPa = 765.102 mmHg
  if (IGATE_CALLSIGN.substring(0, 1) == "R" ) pressValue = pressValue * 0.7501;  // 1 Ettopascal = 0.7501 Millimetri di mercurio
  return String(pressValue,1);



  //if (BM_sensor_status) return String(pressValue,1);
  //else return "N/A";
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
  //do
    //{
      carMenu = 0;
      righello();
      Serial.println(F("\n.. CONFIG MENU ..\n\n   EXAMPLE: for set your call digit 'c' + 'enter'\n   subsequently enter your call + 'enter'\n"));
      righello();
      Serial.println(F("(c) callsign:" ));
      righello();
      Serial.println(F("(1) meteo ssid"));
      Serial.println(F("(2) meteo lat/long"));
      Serial.println(F("(a) meteo altitude"));
      if ( AHTstatus == true ) Serial.println(F("(u) meteo use thermometer"));
      Serial.println(F("(h) meteo drift thermal sensor"));
      Serial.println(F("(r) meteo drift pressure sensor"));
      Serial.println(F("(4) meteo info"));
      Serial.println(F("(s) send data meteo modality"));
      righello();
      Serial.println(F("(5) digi/iGate ssid"));
      Serial.println(F("(6) digi/iGate lat/long"));
      Serial.println(F("(g) digi/iGate info"));
      righello();
      Serial.println(F("(d) digi on/off"));
      Serial.println(F("(i) iGate on/off"));
      Serial.println(F("(b) backup fault iGate to digi on/off"));
      righello();
      Serial.println(F("(t) APRS beacons tx interval"));
      Serial.println(F("(8) APRS-IS passcode"));
      Serial.println(F("(9) APRS-IS server"));
      righello();
      Serial.println(F("(n) Wifi on/off"));
      Serial.println(F("(w) Wifi ssid"));
      Serial.println(F("(7) Wifi password"));
      righello();
      Serial.println(F("(f) LoRa frequency"));
      Serial.println(F("(p) LoRa power"));
      righello();
      Serial.println(F("(y) display OLED on/off"));
      righello();
      Serial.println(F("(0) EXIT"));
      righello();

    do
      {

    
      carMenu = readCarMenu();
      switch (carMenu)
        {
          case '0' :
            verifica_parametri();
            if (Use_WiFi) {
              if (Tmp == "start server") start_server();
              if (!check_wifi()) WiFi_setup();
            }
            status_display();
            make_display();
          break;


          case 'h' :
              Serial.print(F("thermal drift (max +/- 5) | ex: -0.25"));
              readCharArray(drift_thermC);
              if (atof(drift_thermC) >5 || atof(drift_thermC) < -5 )
                {
                  Serial.println(F(" = ERROR"));
                  break;
                }
              if (ptr>5 ) ptr=5;
              EEPROM_writer(47,47+ptr-1,drift_thermC);
              EEPROM_eraser(47+ptr,51);
              Serial.print(F(" = "));
              drift_therm = atof(drift_thermC);  
              Serial.println(drift_therm, 2);   
              BottomBanner();
          break;


          case 'r' :
            Serial.print(F("pressure drift (max +/- 10 ) | ex: -2"));
            readCharArray(tmp_buffer);
            if (atoi(tmp_buffer) >=-10 && atoi(tmp_buffer) <= 10 )
              {
                drift_pres = atoi(tmp_buffer);
                Serial.print(F(" = "));
                Serial.println(drift_pres);
                EEPROM.write(169, (drift_pres+10));
                EEPROM.commit();
                BottomBanner();
                break;
              }
            else 
              {
                Serial.println(F(" = ERROR"));
                BottomBanner();
                break;
              }


          case 'c' :
            Serial.print(F("callsign - ex IZ1XYZ"));
            readCharArray(call);
            if (ptr > 6) ptr = 6;
            EEPROM_writer(6,6+ptr-1,call);
            EEPROM_eraser(6+ptr,11);
            Serial.print(F(" = "));
            Serial.println(call);
            BottomBanner();
          break;


          case 'y' :
            OLED_swapp();
            BottomBanner();
          break;

       
          case 'b' :
            backupigateSwitch = !backupigateSwitch;
            Serial.print(F("backup is: "));
            if (backupigateSwitch ) Serial.println(F("ON"));
            else Serial.println(F("OFF"));
            EEPROM.write(165, backupigateSwitch);
            EEPROM.commit(); 
            make_display;
            BottomBanner();
          break;
   
         
          case 's' :
            Serial.print(F("0=disable | 1=RF | 2=ip | 3=RF+ip - ex: 2"));
            readCharArray(tmp_buffer);
            meteo_tx_mode = atoi(tmp_buffer);
            
            Serial.print(F(" = "));
            Serial.print(meteo_tx_mode);
            if ( meteo_tx_mode == 0 ) {
              Serial.println(F(" = disabled"));
              aprsis.stop();
            }  
            if ( meteo_tx_mode == 1 ) Serial.println(F(" = send to RF"));
            if ( meteo_tx_mode == 2 ) Serial.println(F(" = send to ip"));
            if ( meteo_tx_mode == 3 ) Serial.println(F(" = send to RF+ip"));
            verifica_parametri();
            EEPROM.write(164, meteo_tx_mode);
            EEPROM.commit();
            make_display();
            BottomBanner();
          break;
      

          case 'i' :
            igateSwitch = !igateSwitch;
            Serial.print(F("iGate is: "));
            if (igateSwitch )Serial.println(F("ON"));
            else Serial.println(F("OFF"));
            if (igateSwitch) EEPROM.write(166, 1);
            else EEPROM.write(166, 0);
            EEPROM.commit(); 
            make_display();
            aprsis.stop();      // a ogni variazione stoppalo a prescindere, puo' essere che APRS-IS fosse già connesso per la meteo
            BottomBanner();     // e la regola é che in prima battuta ci si logga con il CAll dell' iGate e con quello della meteo
                                // in seconda istanza se iGate non é attivo. Se iGate viene attivato ci si ri-logga sempre
          break;


            case 'n' :
            Use_WiFi = !Use_WiFi;
            Serial.print(F("WiFi is: "));
            if ( Use_WiFi ) Serial.println(F("ON"));
            if ( !Use_WiFi ) Serial.println(F("OFF"));
            if ( Use_WiFi ) Tmp = "start server";
            EEPROM.write(59, Use_WiFi);
            EEPROM.commit(); 
            make_display();
            BottomBanner();
            
            break;
 

          case 'd' :
            digiSwitch = !digiSwitch;
            Serial.print(F("digipeater is: "));
            if (digiSwitch ) Serial.println(F("ON"));
            else Serial.println(F("OFF"));
            EEPROM.write(167, digiSwitch);
            EEPROM.commit(); 
            make_display();
            BottomBanner();
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
            BottomBanner();
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
            BottomBanner();
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
            BottomBanner();
          break;   
  

          case '2' :
            Serial.print(F("lat,long meteo - ex: 78.7562,18.5162"));
            readCharArray(tmp_buffer);
        
            if ( ptr > 22 || sep == 0)
              {
                Serial.print(F(" .. error .. \n"));
                BottomBanner();
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
            BottomBanner();
          break; 
            

          case '6' :
            Serial.print(F("lat,long igate - ex: 78.7562,18.5162"));
            readCharArray(tmp_buffer);
        
            if ( ptr > 22 || sep == 0)
              {
                Serial.println(F(" .. error .. \n"));
                BottomBanner();
                break;
              }            
            array_eraser(0,9,lat_igate);
            array_eraser(0,10,lon_igate);
            EEPROM_eraser(235,255);   // cancella latitudine e longitudine
            tmp=235;
            while (tmp != 235+sep) // -- salva lat igate
              {
                  EEPROM.write( tmp, tmp_buffer[tmp-235] ); // si scrive a partire dalla cella 235 fino alla 244
                  EEPROM.commit();
                  delay(30);
                  lat_igate[tmp-235]=tmp_buffer[tmp-235];
                  tmp++;
              } 
           
            tmp=245;
            while (tmp != 245+ptr-sep-1) // -- salva lon igate
              {
                  EEPROM.write( tmp, tmp_buffer[tmp-245+sep+1] ); // si scrive a partire dalla cella 245 fino alla 255
                  EEPROM.commit();
                  delay(30);
                  lon_igate[tmp-245]=tmp_buffer[tmp-245+sep+1];
                  tmp++;
              }           
                             
            Serial.print(F(" = "));
            Serial.print(atof(lat_igate),6);
            Serial.print(F(","));
            Serial.println(atof(lon_igate),6);
            verifica_parametri();         // calcola in notazione APRS
            BottomBanner();
          break;
      
          
          case 'a' :
            Serial.print(F("altitude - ex: 445"));
            readCharArray(altitude);
            if (ptr > 4) ptr = 4;
            EEPROM_writer(35,35+ptr-1,altitude);
            EEPROM_eraser(35+ptr,38);
            Serial.print(F(" = "));
            Serial.println(altitude);
            BottomBanner();
          break;


          case 'w' :
            Serial.print(F("WiFi ssiD [ max 20 char ]"));
            readCharArray(WiFi_ssiD);
            if ( ptr>20 ) ptr=20; 
            EEPROM_writer(170,170+ptr-1,WiFi_ssiD);
            EEPROM_eraser(170+ptr,189);
            Serial.print(F(" = "));
            Serial.println(WiFi_ssiD);
            BottomBanner();
          break;


          case '7' :
            Serial.print(F("WiFi password [ max 25 char ]"));
            readCharArray(WiFi_pwd);
            if (ptr>50 ) ptr=50;
            EEPROM_writer(190,190+ptr-1,WiFi_pwd);
            EEPROM_eraser(190+ptr,214);
            Serial.print(F(" = "));
            Serial.println(WiFi_pwd);
            BottomBanner();
          break;


          case '9' :
            Serial.print(F("APRS server - ex: rotate.aprs2.net"));
            readCharArray(aprs_server);
            if (ptr>20 ) ptr=20;
            EEPROM_writer(215,215+ptr-1,aprs_server);
            EEPROM_eraser(215+ptr,234);
            Serial.print(F(" = "));
            Serial.println(aprs_server);
            APRSISServer = String(aprs_server);
            BottomBanner();
          break;


          case '4' :
            Serial.print(F("meteo info - ex: .. WXmeteo LoRa tech .."));
            readCharArray(meteo_info);
            if (ptr>50 ) ptr=50;
            EEPROM_writer(60,60+ptr-1,meteo_info);
            EEPROM_eraser(60+ptr,111);
            Serial.print(F(" = "));
            Serial.println(meteo_info);
            BottomBanner();
          break;


          case '8' :
            Serial.print(F("APRS passcode - ex: 19617"));
            readCharArray(aprs_passcode);
            EEPROM_writer(53,53+4,aprs_passcode);
            Serial.print(F(" = "));
            Serial.println(aprs_passcode);
            BottomBanner();
          break;
      
          case 'g' :
            Serial.print(F("igate info - ex: .. iGate LoRa tech .."));
            readCharArray(igate_info);
            if (ptr>50 ) ptr=50;
            EEPROM_writer(112,112+ptr-1,igate_info);
            EEPROM_eraser(112+ptr,163);
            Serial.print(F(" = "));
            Serial.println(igate_info);
            break;

          case 'f' :
            Serial.print(F("frequency - ex: 433775"));
            readCharArray(frequencyC);
            EEPROM_writer(41,46,frequencyC);
            Serial.print(F(" = "));  
            Serial.println(frequencyC);
            BottomBanner();
          break;


          case 'p' :
            Serial.print(F("power 2-17 dBm - ex: 17"));
            readCharArray(tmp_buffer);
            LoRa_power = atoi(tmp_buffer);
            verifica_parametri();
            Serial.print(F(" = "));
            Serial.println(LoRa_power);
            LoRa.setTxPower(LoRa_power);
            delay(1000);
            EEPROM.write(39, LoRa_power);
            EEPROM.commit();
            BottomBanner();
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
            BottomBanner();
          break;
         
         }        
  } while (carMenu != '0' );
  //Serial.println();
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


void BottomBanner()
{
  tmp=0;
  while (tmp != 54)
    {
      Serial.print(F("- "));
      tmp++;  
    }
    Serial.println(F("(0) EXIT"));
    tmp=0;
}


 /*
  ---------------------------------------------------------------------------
    STATUS SERIAL DISPLAY 
  ---------------------------------------------------------------------------
  */

void status_display()
  {
    make_display();
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
    Serial.print(F("pressure drift: ")); Serial.print(drift_pres);  Serial.println(F(" hPa"));
    
    /*
    Serial.print(F("meteo: "));
    if (meteoSwitch) Serial.println(F("enabled"));
    else Serial.println(F("disabled"));
    */

    Serial.print(F("send data meteo: "));
    if (meteo_tx_mode == 0 ) Serial.println(F("not enabled"));
    if (meteo_tx_mode == 1 ) Serial.println(F("to RF"));
    if (meteo_tx_mode == 2 ) Serial.println(F("to ip"));
    if (meteo_tx_mode == 3 ) Serial.println(F("to RF+ip"));

    righello();
    
    Serial.print(F("iGate call: "));
    Serial.print(call);
    Serial.print(F("-"));
    Serial.println(igate_ssiD);

    Serial.print(F("iGate lat,long: "));
    Serial.print(atof(lat_igate),6);
    Serial.print(F(" , "));
    Serial.println(atof(lon_igate),6);

    Serial.print(F("iGate/digipeater info: "));
    Serial.println(igate_info);

    Serial.print(F("iGate: "));
    if (igateSwitch ) 
      {
        Serial.println(F("enabled"));
        Serial.print(F("APRS passcode: "));Serial.println(aprs_passcode);
        Serial.print(F("APRS server: ")); Serial.print(aprs_server);
        if (APRS_login) Serial.println(F(" - connected")); 
        else Serial.println(F(" - NOT connected")); 
        
      }
    else Serial.println(F("not enabled"));
        
    Serial.print(F("digipeater: "));
    if (digiSwitch ) Serial.println(F("enabled"));
    else Serial.println(F("not enabled"));
    Serial.print(F("backup iGate to digipeater: "));
    if (backupigateSwitch) Serial.println(F("enabled"));
    else Serial.println(F("not enabled"));

    righello();
    Serial.print(F("beacons tx interval: ")); Serial.print(tx_interval); Serial.println(F(" minutes"));
   
    
    righello();
    Serial.print(F("LoRa frequency: ")); Serial.print(frequencyC); Serial.println(F(" KHz"));
    Serial.print(F("LoRa power: ")); Serial.print(LoRa_power); Serial.println(F(" dBm"));
    //if (Xmode && Experimental) Serial.println(F("LoRA speed mode enabled"));
   

 
    righello();
    if (!Use_WiFi ) Serial.println(F("Wifi: not enabled")); 
    else
      {
        Serial.print(F("Wifi ssid: ")); Serial.println(WiFi_ssiD);
        Serial.print(F("Wifi password: "));  Serial.println(WiFi_pwd);
        Serial.print(F("ip: "));  Serial.println(myIP);
      }      

  
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
      cerca alla locazione 58 il valore della release
    */ 
    if (EEPROM.read(58) != int(atof(Release)*100) ) 
      {
        Serial.println(F("initial reset required  - please wait")); 
        initial_reset();  // carica valori di default nella EEPROM
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
    Use_WiFi = EEPROM.read( 59 );  
    meteo_tx_mode = EEPROM.read( 164 );
    backupigateSwitch = EEPROM.read( 165 );

    if (EEPROM.read( 166 ) > 0 )igateSwitch = true;   // Use_IGATE
    else igateSwitch = false;

    if (EEPROM.read( 167 ) > 0 ) digiSwitch = true;   // USE_DIGIPEATER
    else digiSwitch  = false;

    oledSwitch = EEPROM.read( 168 ); // display oled on/off
    drift_pres = EEPROM.read( 169 )-10; // drift_pres

    EEPROM_loader(60,111,meteo_info);
    EEPROM_loader(112,163,igate_info);

    EEPROM_loader(170,189,WiFi_ssiD);
    EEPROM_loader(215,234,aprs_server);
    EEPROM_loader(190,214,WiFi_pwd);
    EEPROM_loader(235,244,lat_igate);
    EEPROM_loader(245,255,lon_igate);
  }


  /*
  ---------------------------------------------------------------------------
    RESET INIZIALE
  ---------------------------------------------------------------------------
  */

void initial_reset()
  {
    
    EEPROM_eraser(0,255);
    Serial.println(F("EEPROM erased - please wait")); // pone a valore '254 tutte le celle

    EEPROM.write(58, int(atof(Release)*100));  // current value of realise

    EEPROM_writer(0, 5,Build);   
    
    char buff0[7]="N0CALL"; 
    EEPROM_writer(6, 11,buff0);
    
    char buff1[3]="25"; 
    EEPROM_writer(35, 35+1,buff1);
    EEPROM_writer(41, 46,frequencyC);       // frequenza dalla cella 41 fino alla 46
    
    char buff2[24]=".. WXmeteo LoRa tech .."; // buff3 = 23 char [ 0--> 22 ]
    EEPROM_writer(60,   60+22,buff2);

    char buff3[24]=".. iGate LoRa tech .."; // buff4 = 23 char [ 0--> 22 ]
    EEPROM_writer(112, 112+22,buff3);

    char buff4[4]="123";                  // buff2 = 3 char [ 0--> 4 ]
    EEPROM_writer(170, 170+2,buff4);      // WiFi SSiD
    EEPROM_writer(190,190+2,buff4);       // WiFi password
    
    char buff5[17]="rotate.aprs2.net";
    EEPROM_writer(215,215+15,buff5);       // APRS server
    
    char buff6[6]="19617";
    EEPROM_writer( 53, 53+4,buff6);       // APRS passcode
    
    char buff7[2]="0";
    EEPROM_writer( 47, 51,buff7);       // drift termometro 
    
    EEPROM.write(12, 3);
    EEPROM.write(13, 10);  
    EEPROM.write(39, 2);  // LoRa power
    EEPROM.write(40, 10); // tx interval
    
    EEPROM.write(59, 0);  // Use_WiFi
    EEPROM.write(164, 0); // meteo_tx_mode
    EEPROM.write(165, 0); // backupigateSwitch
    EEPROM.write(166, 0); // switch igate
    EEPROM.write(167, 0); // switch digi
    EEPROM.write(168, 0); // oled on
    EEPROM.write(169, 10);// drift_pres - 10 equivale a un drift di 0hPA

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
    
    if (meteo_ssiD >99)   meteo_ssiD = 3;
    if (meteo_ssiD <1)    meteo_ssiD = 3;
    if (igate_ssiD >99)   igate_ssiD = 10;
    if (igate_ssiD <1)    igate_ssiD = 10;
    
    if (meteo_tx_mode > 3) meteo_tx_mode = 0;
   
    if (tx_interval <1 )  tx_interval = 10;
    if (tx_interval >20 ) tx_interval = 20;
    
    //hum_lect = time_data_period/(tx_interval*60000);
    //Serial.print("hum_lect: ");
    //Serial.println(hum_lect);

    drift_therm=atof(drift_thermC);
    if ( drift_therm > 5 ) drift_therm = 0;
    if ( drift_therm < -5 ) drift_therm = 0;

    if ( drift_pres > 10 ) drift_pres = 0;
    if ( drift_pres < -10 ) drift_pres = 0;


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

    if (LoRa_power <2 )   LoRa_power = 2;
    if (LoRa_power >20 )  LoRa_power = 20;
    if (!digiSwitch && !igateSwitch && meteo_tx_mode == 0 ) {
      LoRa.sleep();
      Serial.println(F("*** LoRa module set to sleep mode ***"));
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
    while (start <= stop && start != 255) // -- cancella EEPROM 
      {
        //Serial.print("eraser indirizzo: ");Serial.println(start);
        EEPROM.write( start, 254 ); 
        start++;
      }
    if (stop == 255 ) 
      {
        EEPROM.write( 255, 254 ); 
        //Serial.print("eraser indirizzo: ");Serial.println(255);
      }  
    EEPROM.commit(); 
  }


void EEPROM_writer(byte start, byte stop,char tmp_data[50])
  {
    tmp=0;
    while (tmp+start <= stop) // -- scrivi EEPROM 
      {
        //Serial.print("indirizzo: ");Serial.print(tmp+start);
        //Serial.print(" - dato: ");Serial.println(tmp_data[tmp]);
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





void make_display()
  {
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(0,0);
    display.print("iGate/digi:");
    display.print(call);
    display.print("-");

    display.print(igate_ssiD);
    display.setCursor(0,9);

        display.print("iGate:");
        if (igateSwitch )display.print("ON");
        if (!igateSwitch) display.print("OFF");
        if (!backupigateSwitch) display.print(" - digi:");
        if (backupigateSwitch) display.print(" * digi:");
        if (digiSwitch )display.print("ON");
        if (!digiSwitch) display.print("OFF");

    //if (Xmode && Experimental) display.println(" X");
    //else display.println("");

    display.setCursor(0,18);
    display.print("meteo send ");
    if (meteo_tx_mode == 0) display.println("disabled");
    if (meteo_tx_mode == 1) display.println("to RF");
    if (meteo_tx_mode == 2) display.println("to ip");
    if (meteo_tx_mode == 3) display.println("to RF+ip");
    
    display.setCursor(0,27);
    if (Use_WiFi) {
      display.print("ip: ");
      display.println(myIP);
    } else display.print("WiFi not enabled");
    
    display.setCursor(0,36);
    display.println("---------------------");
    display.setCursor(0,45);
    display.print("project by OK2DDS");
    display.setCursor(0,54);
    display.print("modified by IW1CGW");
    display.display();  
  }


  void OLED_swapp()
  {
    oledSwitch = !oledSwitch;
    Serial.print(F("display oled is:"));
    if (oledSwitch )
      {
        Serial.println(F("OFF"));
        display.dim(true);
      }
    else
      {
        Serial.println(F("ON"));
        display.dim(false);
      }
    EEPROM.write(168, oledSwitch);
    EEPROM.commit(); 
    display.display();
  }



void WiFi_setup()
{
    Serial.println("WiFi not connected!");
    WiFi_setup_retry=0;
    wifiStatus = false;
    myIP="0.0.0.0";
    WiFi_login_retry=millis();
    make_display();
    while (WiFi_setup_retry < 4 ) 
      {
        WiFi_setup_retry++;
        Serial.print("Connecting to WiFi.. retry:");
        Serial.println(WiFi_setup_retry);
        WiFi.setAutoReconnect(true);
        WiFi.setHostname(Hostname);
        WiFi.begin(WiFi_ssiD, WiFi_pwd);
        delay(1500);
        if ( WiFi.status() == WL_CONNECTED ) WiFi_setup_retry = 99;
        delay(500);
      }
    if ( WiFi.status() == WL_CONNECTED )
     { 
        wifiStatus = true;
        myIP = ipToString(WiFi.localIP());
        Serial.println("IP:" + myIP);
        make_display();
      }
    else Serial.println("Unable to connect to WiFi.. 60 seconds and try again");

 }



 /*
  ---------------------------------------------------------------------------
    GRAPHIC DISPLAY 
  ---------------------------------------------------------------------------
  */
  
void make_meteo_display()
  {
    //if (!meteoSwitch) exit;
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(1);
    byte sep = 0;
    
    //if ( BM_sensor_status )
      {
        
            float temp = getTempC();
            String stemp = String(temp,1);
            float press = getPressure();
            String spressd = String(press,0);
            float Hum = getHum();
            String sHum = String(Hum);
        
      
        display.setCursor(0,sep);
        display.print("Temperature:");
        display.print(stemp);
        display.println(" C");
        sep=sep+12;

        display.setCursor(0,sep);
        display.print("Pressure   :");
        display.print(spressd);
        display.println(" hPa");
        sep=sep+12;

        display.setCursor(0,sep);
        display.print("Humidity   :");
        display.print(sHum);
        display.println(" %");
        sep=sep+12;
      }

    display.setCursor(0,sep);
    display.println("---------------------");
    display.setCursor(0,sep+12);
    display.print(METEO_CALLSIGN);
     
    display.setCursor(0,57);    //fisso fondo schermo dx
    display.print("ip:");
    myIP = ipToString(WiFi.localIP());
    display.print(myIP);
    display.display();  
  }



void start_server() {
 if (Use_WiFi ) 
  {
    Serial.println(F("start server http"));
    Tmp="";
    WiFi_setup();
    server.begin();             
    ws.onEvent(onWsEvent);
    serverWS.addHandler(&ws);
    serverWS.begin();
    delay(500);
    if (igateSwitch && check_wifi() || meteo_tx_mode > 1 && check_wifi()) aprsis_connect();
    delay(500);
  }
}









bool checkForUpdates() {
  if ( check_wifi() ) {
  HTTPClient http;
  Tmp = String(OTA_web_server) + (OTA_fileName_ver);            
  http.begin(Tmp);
  int httpCode = http.GET();
  if (httpCode == 200) {
      String input = http.getString();
      //JsonDocument doc;
      //deserializeJson(doc, input);
      http.end();
      
      //String last_Build = doc[String("build")];
      //String file = doc[String("file")];
      //Update_path = String(OTA_web_server) += file;
            
      
      String last_Build;
      last_Build = input;
            
      /*
      String last_Project = doc[String("project")];
      String last_realise = doc[String("realise")];
      String hardware_model = doc[String("hardware_model")];
      String checksum = doc[String("checksum")];
      //Serial.print("project: ");Serial.println(project);
      //Serial.print("last_realise: ");Serial.println(last_realise);
      //Serial.print("last_build: ");Serial.println(last_build);
      //Serial.print("hardware_model: ");Serial.println(hardware_model);
      Serial.print("file: ");Serial.println(file);
      //Serial.print("checksum: ");Serial.println(checksum);
      */

        display.clearDisplay();
        display.setTextSize(2);
        display.setCursor(10,10);

     if ( last_Build == Build ) {
        
        /*
        Serial.println(F("OTA: this version is updated"));
        display.print("version");
        display.setCursor(10,35);
        display.print("updated");
        display.display(); 
        delay(3000);
        */
        OTA_code=0;
        return false;
      } 
      else {
        Serial.println(F("OTA: an update is available"));
        display.print("update");
        display.setCursor(10,35);
        display.print("available");
        display.display(); 
        delay(2000);
        OTA_code=1;
        return true;
      }     
   }
   else { 
    http.end();
    //Serial.print(F("OTA: error fetching, error code: "));Serial.println(httpCode);
    OTA_code=2;
    return false;
   }
   } else return false;


} 


bool updateFirmware()
  {
    if ( check_wifi() ) {
    
    HTTPClient http;
    http.begin(String(OTA_web_server) + String(OTA_fileName_update));
    int httpCode = http.GET();
    if (httpCode == 200)
      {
        Serial.println(F("OTA: install update"));
        display.clearDisplay();
        display.setCursor(10,10);
        display.print("install");
        display.setCursor(10,35);
        display.print("update");
        display.display(); 
        int contentLength = http.getSize();
        WiFiClient &client = http.getStream();

        if (Update.begin(contentLength))
          {
            size_t written = Update.writeStream(client);
            http.end();
            if (written == contentLength)
              {
                Serial.println("OTA: written: " + String(written) + " successfully");
                display.clearDisplay();
                display.setCursor(10,10);
                display.print("update");
                display.setCursor(10,35);
                display.print("ok");
                display.display(); 
                delay(2000);
                OTA_code=3;
              } else
              {
                Serial.println("OTA: error: write failed, written only: " + String(written));
                OTA_display_ko();
                OTA_code=4;
                return false;
              }

            if (Update.end())
              {
                 Serial.println(F("OTA: done!"));
                 if (Update.isFinished())
                  {
                    Serial.println(F("OTA: update successfully completed, now rebooting."));
                    display.clearDisplay();
                    display.setCursor(10,10);
                    display.print("reboot");
                    display.setCursor(10,35);
                    display.print("now");
                    display.display(); 
                    delay(2000);
                    OTA_code=5;
                    OTA_logbook(); 
                    ESP.restart();
                    return true;
                  } else {
                            Serial.println(F("OTA: error: update not finished"));
                             OTA_code=6;
                            return false;
                            OTA_display_ko();
                          }
                } else  {
                          Serial.println("OTA: error: " + String(Update.getError()));
                           OTA_code=7;
                          OTA_display_ko();
                          return false;
                        }
            } else {
                      http.end();
                      Serial.println(F("OTA: error: not enough space to begin"));
                       OTA_code=8;
                      OTA_display_ko();
                      return false;
                    }
      } else  {
                http.end();
                Serial.println(F("OTA: error: failed to fetch firmware"));
                 OTA_code=9;
                OTA_display_ko();
                return false;
              }
} else return false;

}

void OTA_display_ko()
{
                display.clearDisplay();
                display.setCursor(10,10);
                display.print("update");
                display.setCursor(10,35);
                display.print("ko");
                display.display(); 
                delay(2000);
}




bool NTP_query()
{
  if ( check_wifi() ) {
    const char* ntpServer = "time.nist.gov";
    const long  gmtOffset_sec = 0;
    const int   daylightOffset_sec = 3600;

    // Init and get the time
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    delay(1000);
    if(!getLocalTime(&timeinfo)){
      //Serial.println("NTP: Failed to obtain time");
       return false;
    }
    sprintf(NTP_data, "%04d-%02d-%02d %02d:%02d:%02d", timeinfo.tm_year+1900, timeinfo.tm_mon+1, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    return true;
} else return false;
}


void OTA_logbook()
{
    //--- coming soon
}



bool vpn(){
 //Husarnet.join(husarnetJoinCode, hostName);
 //Husarnet.start();
 return true;
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
58 - 58 // *** VERIFY VERSION *** | ex: value 102 if for Release "1.02" [ int(atof(Release)*100) ]
59 - 59 // Use_WiFi| bool

60 - 111 // info meteo: .. WXmeteo Alma Frabosa - 650m .. | 50 caratteri + flag fine testo - carattere cella 111 -
112 - 163 // info igate: .. WXmeteo Alma Frabosa - 650m .. | 50 caratteri + flag fine testo - carattere cella 163 -

164 - 164 // meteo_tx_mode | 1 = in APRS-IS [meteoAPRSswitch] | 2 = in LoRa [meteoSwitch] | 0  = disable 
165 - 165 // switch backup for fault igate [backupigateSwitch]
166 - 166 / /switch igate [igateSwitch]
167 - 167 // switch digipeater [digiSWitch]
168 - 168 // display on/off
169 - 169 // pres trim

170 - 189 // WiFi ssiD [ 20 caratteri ]
190 - 214 // WiFi password [ 25 caratteri ]
215 - 234 // Aprs_server [ 20 caratteri ]
235 - 244 // latitude igate | 10 caratteri
245 - 255 // longitudine igate | 11 caratteri 



DIGIPEATERS
/# - Generic digipeater
1# - WIDE1-1 digipeater
A# - Alternate input (i.e. 144.990MHz) digipeater
E# - Emergency powered (assumed full normal digi)
I# - I-gate equipped digipeater
L# - WIDEn-N with path length trapping
P# - PacComm
S# - SSn-N digipeater (includes WIDEn-N)
X# - eXperimental digipeater
V# - Viscous https://github.com/PhirePhly/aprx/blob/master/ViscousDigipeater.README
W# - WIDEn-N, SSn-N and Trapping

GATEWAYS: #&
/& = HF Gateway  <= the original primary table definition
I& = Igate Generic (please use more specific overlay)
L& - Lora Igate
R& = Receive only IGate (do not send msgs back to RF)
P& = PSKmail node
T& = TX igate with path set to 1 hop only)
W& = WIRES-X as opposed to W0 for WiresII
2& = TX igate with path set to 2 hops (not generally good idea)


*/