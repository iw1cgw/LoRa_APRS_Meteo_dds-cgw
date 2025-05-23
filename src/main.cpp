#include "config.h"
#include <LoRa.h>
/*
Chip LoRa	Banda di Frequenza
SX1276	868 MHz / 915 MHz
SX1278	433 MHz
*/


unsigned long millis_token_tx;
boolean token_tx;
#include "website.h" 


#include "AsyncWebSocket.h"   // modified to: #define WS_MAX_QUEUED_MESSAGES 15 -- portato a 128
#include "AsyncWebSocket.cpp" // modified to: #define WS_MAX_QUEUED_MESSAGES 15 -- portato a 128

#include <ESPAsyncWebServer.h>

#include <Arduino.h>
#include <driver/pcnt.h>

#include <Wire.h>
//#include <WiFi.h>
//#include <WiFiAP.h>
//#include <ESP32Ping.h>
//#include <WiFiClient.h>
#include <Update.h>

#include <esp_wifi.h>
#include <ESP32Time.h>

int GMT_zone;
ESP32Time rtc(0);     // offset in seconds GMT - lasciare a '0' il valore corrente verra dato dal server NTP
String LogEvent;

unsigned int sensor_refresh;
//unsigned int APRSIS_login_retry_millis;

//----------------------------- DTH22

#include <ErriezDHT22.h>
// Create DHT22 sensor object
DHT22 dht22 = DHT22(DHT22_PIN);

//----------------------------- DTH22

#include <AsyncTCP.h>
#include <ESP32_FTPClient.h>
#include <HTTPClient.h>

String generateEncodedTelemetry();
String generateEncodedTelemetryBytes(int tempValue);
String telemetry;
String encodedBytes;

void lora_send(String tx_data);
void lora_setup();
void setup_WiFi(String WiFi_ssiD , String WiFi_pwd );

//----- variabili APRS-IS
boolean aprs_login = false;          // indica se effettuato o meno corretto login a APRS-IS
unsigned long aprs_login_millis;  // orologio per verifica della effettiva connessione al server APRS-IS basandosi sul ritorno del banner
void aprsis_setup();            // oprazioni di connessioneal server APRS-IS
void aprsis_send(String aprsis_packet);
String current_server_aprsis;
boolean hide_aprs_server;
char aprs_passcode[50] = "19617";
//-----------------------------------
WiFiClient aprsis;
String APRSISServer;
//-----------------------------------


#include <Adafruit_GFX.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_BME280.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>//https://github.com/espressif/arduino-esp32/tree/master/libraries/EEPROM

int16_t count = 0;


//-----------------------------------------------
void calc_dist(String rxPacket, int pos2);
int max_digi_radius;
int Km;
int LastKm;
#include"haversine.h"
//-----------------------------------------------

//-----------------------------------------------
#include <esp_task_wdt.h> 
#define WDT_TIMEOUT 270  // secondi entro i quali resettare il watchdog
//----------------------------------------------- 

//-----------------------------------------------
#include <INA226_WE.h>   
#define I2C_ADDRESS 0x40
INA226_WE ina226 = INA226_WE(I2C_ADDRESS);
boolean INA226_status = LOW;
float shuntVoltage_mV = 0.0;
float loadVoltage_V = 0.0;
float busVoltage_V = 0.0;
float current_mA = 0.0;
//float power_mW = 0.0;
int tlm_voltage;
int tlm_current;
//-----------------------------------------------
char aprs_server[50]="rotate.aprs2.net";
char wunderid[50]="";
char wunderpwd[50]="";
bool wunderSwitch;
byte wunderSendStatus=0;    // 0 indefinito - 1 ok - 9 ko
void wunder_send();
void make_blink();
void save_MinMax();

int cntValue;

String Tmp;
bool is_setup;
boolean Pled;
boolean autoreboot;
unsigned long Pled_millis;
unsigned long tmp_dash_pwd_millis;

byte pkRx10m; // pacchetti ricevuti da LoRa nei 10 minuti [ pacchetti validi ]
byte pkTx10m; // pacchetti trasmessi da LoRa nei 10 minuti [ come digipeater ]
//byte pkig10m; // pacchetti gestiti da igate nei 10 minuti;

String MHeard[21]; 

char digi_route[50]="";
char digi_banned[50]="";

bool AP_auto_shutdown = false;
bool AP_active = true;

void beacon_telemetry_param();
void beacon_meteo();
void beacon_meteo_status();
void beacon_igate_status();
void beacon_igate();
bool checkForUpdates();
bool updateFirmware();
bool token_verify_update;

void OTA_display_ko();
void OTA_logbook();
bool OTA_enabled;
bool NTP_query();
bool RTC_status;
struct tm timeinfo;
char NTP_data[20];
char NTP_dataLight[10];
byte OTA_code;

unsigned long refresh_millis; //--- refresh automatico della pagina web
unsigned long dashboard_activity_millis;  //-- timer attività sulla dashboard

unsigned int H;   // ore di funzionamento

uint8_t retr;           // multiuso uso prevalente per contare i retry durante le connessioni ai server
byte ptr;               // multiuso uso prevalente contatore caratteri digitati da menu seriale
byte sep;               // multiuso uso prevalente indica posizione carattere separatore delle stringhe lat/long
unsigned int tmp;       // multiuso
char tmp_buffer[60]=""; // multiuso uso prevalente a supporto menu seriale e caricamento EEPROM
char carMenu;           // a supporto seriale e menu
char car;               // a supporto seriale e menu

byte cnt_telem;         // contatore stringhe della telemetria da 0 a 255
byte cnt_param;         // contatore stringhe parametriche della telemetria + status 

unsigned int tmp_Vint;
unsigned int tmp_Vext;

WiFiClient wunder;

int pktIndex;
int mpktIndex;
int apktIndex;

AsyncWebServer serverWS(5028);
AsyncWebSocket ws("/ws");
WiFiServer server(80);

void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len);
void updateWebSocket();
String HTMLelementDef(String elementID);

bool isWSconnected = false;

String ipToString(IPAddress ip);
String myIP;
bool GETIndex(String header, String requestPath);

unsigned long lastDigipeat = 0;
String lastRXstation = "";
String mem_lastRXstation = "";

float voltage = 0;
float voltage_dashboard = 0;
int battPercent = 0;
unsigned long  millis_led = 0;
//const byte PLED1 = 25;   

float weathervane_voltage = 0;
//float voltage_12 = 0;
float voltage_aux = 0;

unsigned long lastIgBeacon = 0;     // timer del beacon igate
unsigned long lastStIgBeacon = 0;   // timer del beacon status dell'igate 
unsigned long lastStMtBeacon = 0;     // timer del beacon status della meteo
unsigned long lastMtBeacon = 0;     // timer del beacon meteo
unsigned long lastTlParamBeacon = 0;// timer delle stringhe parametriche della telemetria

Adafruit_BME280 bme;
Adafruit_BMP280 bmp;
Adafruit_AHTX0 aht;
bool BM_sensor_status = false;
bool AHTstatus = false;
bool BMPstatus = false;
bool BMEstatus = false;
bool DHT22status = false;

float getTempC();

float TempC;                      // valore corrente del termometro letto solo ogni 2000 millisecondi
unsigned long lastSensorUpdate;   // orologio dell'update dei valori dei sensori e dell'update della dashboard
byte token_sensor;                // ciclo 0/1 per lettura sensori [ se si usa DHT22 é utile per mantenere oltre 2000millisecs tra una lettura e l'altra]

float getHum();
float Hum;          // valore corrente del umidometro letto solo ogni 2000 millisecondi    

float getPressure();
float Press;      // valore corrente del pressione letto solo ogni 2000 millisecondi 

float getDewPoint();   // valore corrente del DewPoint calcolato solo ogni 2000 millisecondi 
double DewPoint;

String getHumAPRS();
String getTempAPRS();
String getPressureAPRS();

String forecast;
bool zambretti;
void zambrettiRoutine();
String pressureTrend = "null";
float pressureTrendReference = 0;
unsigned long pressureTrendTimeout = 0;
String weatherSymbol;

//----------------------

bool USE_weathervane = false;
bool USE_anemometer = false;
bool telemSwitch = false;
bool USE_12V = false;
bool A_SCALE = false;
unsigned int degrees_weathervane; // valore finale gradi weathervane
char anemometer_lenghtC[50]="14";
float anemometer_lenght=14;

String getWindDirAPRS();

float maxWind;
float maxGust;
float minPM;
float maxPM;

float mph(float metersPerSeconds);
String windSpeedAPRS(float fSpeed);

//int windMeterSpins = 0;
int windMeterSpinsInTimeout = 0;
unsigned long windCycleDuration = 0;
unsigned long windTimeout = 0;
unsigned long windLastGust = 0;
float windActualSpeed = 0;
float windKMH(float windMS);
float windKnots(float windMS);
float windLongPeriodSpeed = 0;
float gust = 0;

String tempToWeb(float tempValue);
String DewPointToWeb(float DewPointValue);
String pressToWeb(float pressValue);
String HumToWeb(float HumValue);

String valueForJSON(String value);

String tempValues;
String pressValues;
String HumValues;
String windValues;
String DewPointValues;

float minTemp;
float maxTemp;

float minPress;
float maxPress;

String addGraphValue(String values, String value);
String generateGraph(String values, String graphName, String graphID, int r, int g, int b);

uint8_t meteo_tx_mode = 0;      // 0=disable | 1=RF | 2=ip | 3=RF+ip - ex: 2 - //dalla 20240927 la modalità di invio rf+ip non é più possibile per la stringa meteo

bool igateSwitch = false;       // = USE iGate
bool iGateBeaconOnRF = false;   // beacon dell' igate anche in RF
bool digiSwitch = false;        // = USE_DIGIPEATER

bool backupStatusSwitch = false; // backup su fault igate, inserisce digipeater
void start_Backup();                  // routine che attiva il backup del iGate sul digipeater + relative funzioni meteo
void stop_Backup();               // routine che ripristina il setup originale angte backup
bool backupStatus = false;       //indica se il sistema sta funzionando in backup
unsigned long lost_connection_millis; //--- contatore delle connessioni perse

bool USE_oled = true;           //  oled acceso

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

char igate_info[50]="...";
char meteo_info[50]="...";   

void menu();
void menu_digi();
void righello();
void BottomBanner();
void banner();
void status_display();
void initial_reset();
void load_param();
void make_display();  // aggiornameno display OLED
void make_meteo_display();

void EEPROM_eraser(int start, int stop);
void EEPROM_writer(int start, int stop, char tmp_data[50]);
void EEPROM_loader(int start, int stop, char tmp_data[50]);
void array_eraser(byte start, byte stop, char tmp_data[50]);
void verifica_parametri();

void APRS_LatLon();

char lat_meteo[50] ="";
char lon_meteo[50] ="";

char lat_meteo_APRS[50];
char lon_meteo_APRS[50];

char lat_igate[50] = "";
char lon_igate[50] = "";

char lat_igate_APRS[50];
char lon_igate_APRS[50];

char frequencyC[50]="433775";

byte mod_type=0;                // 212 for poland mode also for standard mode

char WiFi1_ssiD[50]="";
char WiFi1_pwd[50]= "";
char WiFi2_ssiD[50]="";
char WiFi2_pwd[50]= "";
char AP_pwd[50]= "";

char altitude[50]="0";

char call[50]="";
String IGATE_CALLSIGN;
String METEO_CALLSIGN;
byte meteo_ssiD=3;
byte igate_ssiD=10;
byte LoRa_power=2;
//byte LoRa_rx_gain=0;  // auto

byte tx_interval=10;
boolean beacon_disable=false;

int drift_weathervane;
int drift_pres=10;
float drift_therm=0;
char drift_thermC[50]="0";
float drift_battery=0;
char drift_batteryC[50]="0";

//Adafruit_BME280 bme; // use I2C interface
  Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
  Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
  Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();;

sensors_event_t humidity_event, temp_event, pressure_event;

byte ch_term=0;   // termometro in uso 00BMP280 1= AHT20
float minHum;
float maxHum;

//=================================================== deep sleep
boolean switch_sleep = false;
void start_sleep();
void print_wakeup_reason();
RTC_DATA_ATTR int bootCount = 0;
//===================================================

void fAnemometer( void *pvParameters )  // https://forum.arduino.cc/t/solved-esp32-anemometer-on-reed-switch/1090185/11
{
  //int16_t count = 0;
  count = 0;
  for (;;)
  {
    pcnt_get_counter_value(PCNT_UNIT_0, &count);
    //Serial.println(count);
    
    windActualSpeed = float(count * 1000 / (millis() - windCycleDuration)) * anemometer_lenght / 100;
    windCycleDuration = millis();

    windMeterSpinsInTimeout = windMeterSpinsInTimeout+count;
    pcnt_counter_clear(PCNT_UNIT_0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
  vTaskDelete(NULL);
}

#define bottom   ".. (m)enu - (d)isplay - (s)end meteo - (t)ransmit beacon - (r)ead sensor ..\n"
#define DEFAULT_STATUS "https://iw1cgw.wordpress.com/lora/" 

char dash_pwd[50]="admin";
char tmp_dash_pwd[50]="admin";
bool check_pwd();

// -------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------


void setup() {
 


  SPI.begin();
  pinMode(SS, OUTPUT);
  digitalWrite(SS, HIGH);





  pinMode(V12_SENSOR_PIN  , INPUT);
  pinMode(weathervane_PIN , INPUT);
  pinMode (PLed_life, OUTPUT);
  pinMode (PLed_life_int, OUTPUT);
  digitalWrite(PLed_life, HIGH);

  int16_t PCNT_H_LIM_VAL = 300;
  int16_t PCNT_L_LIM_VAL = 0;
  // Anemometer
  pcnt_config_t pcnt_config  = {};
  pcnt_config.pulse_gpio_num = GPIO_NUM_34;// Set PCNT input signal and control GPIOs
  pcnt_config.ctrl_gpio_num  = PCNT_PIN_NOT_USED;
  pcnt_config.channel        = PCNT_CHANNEL_0;
  pcnt_config.unit           = PCNT_UNIT_0;
  // What to do on the positive / negative edge of pulse input?
  pcnt_config.pos_mode       = PCNT_COUNT_INC;   // Count up on the positive edge
  pcnt_config.neg_mode       = PCNT_COUNT_DIS;   // Count down disable
  // What to do when control input is LOW or high?
  pcnt_config.lctrl_mode     = PCNT_MODE_KEEP; // Keep the primary counter mode if LOW
  pcnt_config.hctrl_mode     = PCNT_MODE_KEEP;    // Keep the primary counter mode if high
  // Set the maximum and minimum limit values to watch
  pcnt_config.counter_h_lim  = PCNT_H_LIM_VAL;
  pcnt_config.counter_l_lim  = PCNT_L_LIM_VAL;
  pcnt_unit_config(&pcnt_config); // Initialize PCNT unit
  // default filter value for me was 16 even if no manual setting
  pcnt_filter_enable( PCNT_UNIT_0 );
  pcnt_counter_clear( PCNT_UNIT_0 );
  xTaskCreatePinnedToCore( fAnemometer, "fAnemometer", 10000, NULL, 1, NULL, 1 );

  Serial.begin(SERIAL_BAUD);
  delay(100);
  Serial.println( "\n" + String(Project) + " v." + String(Release) + "\nby IW1CGW inspired by OK2DDS' project\n");
  
  EEPROM.begin(EEPROM_SIZE);
  delay(100);
  load_param(); //--- comprensivo di verifica parametri

  display.begin(SSD1306_SWITCHCAPVCC, 0x3c); 
  display.clearDisplay();
  display.drawBitmap(0, 0, logo, 128, 64, WHITE);
  display.display();
  delay(2000);

  //--------------------------------------------------------------------------------------
  BM_sensor_status = false;
  BMPstatus = false;
  BMEstatus = false;
  AHTstatus = false;
  DHT22status = false;

  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  tmp=9;
   //------------------------------ test del AHT20
  if (!aht.begin()) {}
  else
    {
      Serial.println("AHT20 0x38 OK");
      AHTstatus = true;
      display.setCursor(0,tmp);
      display.print("sensor AHT20 0x38 OK");
      tmp=tmp+9;
    } 
  
  if ( AHTstatus ) {
    Wire.beginTransmission(0x38);
    Wire.write(0xA8);  // Comando per uscire dalla sospensione
    Wire.endTransmission();
    //usleep(10000);  // Attendi almeno 10 ms per stabilizzazione
  } 

  //------------------------------ test del BME280
  if (AHTstatus == false) {
    if (!bme.begin(0x76)) {}
    else {
      BMEstatus = true;
      BM_sensor_status = true;
      Serial.println("BME280 0x76 OK");
      display.setCursor(0,tmp);
      display.print("sensor BME280 0x76 OK");
      tmp=tmp+9;
    }
  }

  //------------------------------ test del BMP280 su 2 possibili indirizzi 0x77 se integrato con AHT20 o 0x76 se stand alone

  if (AHTstatus == true && BMEstatus == false) {
    if (!bmp.begin(0x77)) {}
    else {
      Serial.println("BMP280 0x77 OK");
      BMPstatus = true;
      BM_sensor_status = true;
      display.setCursor(0,tmp);
      display.print("sensor BMP280 0x77 OK");
      tmp=tmp+9;
    }
  }
 
  if (AHTstatus == false && BMEstatus == false) {
    if (!bmp.begin(0x76)) {}
    else {
      Serial.println("BMP280 0x76 OK");
      BMPstatus = true;
      BM_sensor_status = true;
      display.setCursor(0,tmp);
      display.print("sensor BMP280 0x76 OK");
      tmp=tmp+9;
   }
  }

  if (AHTstatus) ch_term = 1;    // se presente un AHT20 usa il suo termometro in luogo di quello del BMP280

  //------------------------------ test DTH22

  dht22.begin(DHT22_NUM_SAMPLES);
  delay(2000);
    // Check minimum interval of 2000 ms between sensor reads
    if (dht22.available()) {
      Serial.println(F("DTH22 OK"));
      DHT22status = HIGH; 
      display.setCursor(0,tmp);
      display.print("sensor DTH22 OK");
      tmp=tmp+9;
    }
  //------------------------------ test DTH22

  //------------------------------  risoluzione conflitti per sensori multipli e determinazione timing di aggiornamento
  if ( BMEstatus) {
    BMPstatus = LOW;
    AHTstatus = LOW;
    DHT22status = LOW;
  }
  if ( AHTstatus ) DHT22status = LOW;

  if ( !DHT22status ) sensor_refresh = sensor_refresh_fast
  if (  DHT22status ) sensor_refresh = sensor_refresh_slow
  //------------------------------  risoluzione conflitti per sensori multipli 

  //------------------------------ test del INA226

  if ( !ina226.init() ) {}
  else {
    Serial.println(F("INA226 0x40 OK"));
    INA226_status = HIGH; 
    display.setCursor(0,tmp);
    display.print("sensor INA226 0x40 OK");
    tmp=tmp+9;
    ina226.setResistorRange(0.1, 1.3); // choose resistor 0.1 Ohm and gain range up to 1.3A
  
    /* If the current values delivered by the INA226 differ by a constant factor
      from values obtained with calibrated equipment you can define a correction factor.
      Correction factor = current delivered from calibrated equipment / current delivered by INA226*/
  
    ina226.setCorrectionFactor(0.93);
    ina226.waitUntilConversionCompleted(); //if you comment this line the first data might be zero
  } 
 
  //------------------------------ test presenza weathervane

  ptr = 1;
  while ( ptr <= 10 ) {                                 // esegue 10 letture del sensore
    tmp_Vext = analogRead(weathervane_PIN );
    delay(10);
    weathervane_voltage = tmp_Vext + weathervane_voltage;
    ptr++;
  }

  if (  weathervane_voltage/10 > 1500 && weathervane_voltage/10 < 4000 ) USE_weathervane = HIGH;
  else USE_weathervane = LOW;

  if (USE_weathervane) {
    Serial.println(F("found wathervane"));
    display.setCursor(0,tmp);
    display.print("found wathervane");
    tmp=tmp+9;
  }
  
  display.setCursor(0,tmp);
  display.println("---------------------");
  tmp=tmp+9;
  display.setCursor(0,tmp);
  display.print("version: ");display.print(Release);display.print(" ");display.print(Build);
  //display.print(Project);
  //tmp=tmp+9;
  //display.setCursor(0,tmp);
  //display.print("build:");
  //display.print(Release);
  //tmp=tmp+9;
  //display.setCursor(0,tmp);
  //display.print("build:");
  //display.print(Build);
  //display.display();

  display.display();    //--- su OLED stato dei sensori

  lora_setup();                           // altri parametri, freq, bW, etc

  //------------------------------------------------------------------------------------------------------------------------
  
  make_blink();

  #define DESTCALL_digi    "APLHI0"      
  #define DESTCALL_meteo   "APLHM0"

  esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G); // Disable 802.11N, which may be buggy in ESP32s - (Also needs #include <esp_wifi.h>)
  
  /*
    Modalità	Definizione	Valore numerico
    WIFI_MODE_NULL	Nessuna modalità	0
    WIFI_MODE_STA	Station (client WiFi)	1
    WIFI_MODE_AP	Access Point	2
    WIFI_MODE_APSTA	Access Point + Station	3
  */

  Tmp = String(call) + "-" + String(igate_ssiD);
  WiFi.hostname(Tmp);
  WiFi.mode(WIFI_AP_STA); // Modalità Dual: AP + STA

  /*
  if ( String(WiFi1_ssiD) != "" ) {
    WiFi.mode(WIFI_STA); // Modalità STA

    //WiFi.mode(WIFI_AP_STA); // Modalità Dual: AP + STA
  }
  else {
    //WiFi.mode(WIFI_AP);   // Modalità solo AP
    AP_auto_shutdown = true;
    WiFi.mode(WIFI_AP_STA); // Modalità Dual: AP + STA
  }
    */


    bool result = WiFi.softAP(Tmp,AP_pwd);    // Configurazione dell'Access Point
    if (result) {
      Serial.print(F("Access Point "));Serial.print(Tmp);
      Serial.print(F(" ready, IP: "));Serial.println(WiFi.softAPIP());
      myIP = ipToString(WiFi.softAPIP());
    } else Serial.println("Error to start AP");


  if ( String(WiFi1_ssiD) != "" ) {
    setup_WiFi( String(WiFi1_ssiD) , String(WiFi1_pwd) );
    if ( WiFi.status() != WL_CONNECTED && String(WiFi2_ssiD) != "") {
      setup_WiFi( String(WiFi2_ssiD) , String(WiFi2_pwd) ); 
    }
  }

  //--------------------------------------------------------------------------------------

  Serial.println(F("start server http"));
  delay(1000); 
  server.begin();  
  delay(1000);          
  ws.onEvent(onWsEvent);
  serverWS.addHandler(&ws);
  serverWS.begin();
  delay(1000);
  make_blink();

  //NTP_query();    //--- con setup ESP32 RTC
  //rtc.setTime(30, 29, 22, 14, 6, 2024);
  /*---------set with NTP---------------*/
  //rtc.setTimeStruct(timeinfo);      // eseguito nella NTP_query

  Serial.println("Startup finished.\n");
  status_display();
  make_display();
  delay(2000);
   
  windCycleDuration = millis();

  display.dim(!USE_oled);  // comandi al contrario ... per accencere l'OLED dare display.dim(LOW)
  display.display();
  token_verify_update=false;

  if (BM_sensor_status) {
    pressureTrendReference = getPressure(); // prima lettura puntuale del valore di pressione
    pressureTrendTimeout = millis();
    pressureTrend = "";
    weatherSymbol = "&#128269;";
  }

  // --- configuriamo il watchdog
  esp_task_wdt_init(WDT_TIMEOUT, true); // --- abilitiamo il watchdog
  esp_task_wdt_add(NULL); 

  lastSensorUpdate = 0;
  
  lastTlParamBeacon = millis() - int(tx_interval*48 * 60000) + 15000; // 10 secondi prima delle stringhe parametriche ogni 8 ore
  lastMtBeacon = millis() - int(tx_interval * 60000     ) + 25000;    // 20 secondi prima del primo beacon meteo [ tempo per disattivare lo sleep mode )]
  lastIgBeacon = millis() - int(tx_interval * 60000     ) + 35000;   // beacon igate/digipeater
  lastStMtBeacon = millis() - int(tx_interval*2 * 60000 ) + 40000;   // lo status del meteo 2 volte il tempo standard della stringa meteo 
  lastStIgBeacon = millis() - int(tx_interval*24 * 60000) + 120000;   // lo status del igate [ ogni 4 ore ]

/*  ===============================================================
    https://randomnerdtutorials.com/esp32-deep-sleep-arduino-ide-wake-up-sources/
    =============================================================== */

  //Increment boot number and print it every reboot
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));
  //print_wakeup_reason();  //---Print the wakeup reason for ESP32
}

//-------------------------------------------------------------------
//-------------------------------------------------------------------
//-------------------------------------------------------------------

void loop() {

  make_blink();

  //-------------------------------------------------- verifica funzionalita server APRS-IS
  if ( aprs_login ) {  
    String aprs_server_rx = (aprsis.readString());
    if ( aprs_server_rx.indexOf("# aprsc") != -1 ) {
      aprs_login_millis = millis();                                       //--- ricevuto banner azzera contatore, OK server APRS-IS per i prossimi 60 secondi
      //Serial.println(aprs_server_rx);
      int GmtIndex = aprs_server_rx.indexOf('T', 1);                   //--- Troviamo la posizione di "GMT" nella stringa
      int SpaceIndex = aprs_server_rx.indexOf(' ', GmtIndex+2);          //--- Troviamo la posizione del primo spazio dopo "GMT"+1 nella stringa
      current_server_aprsis = (aprs_server_rx.substring(GmtIndex+2, SpaceIndex)); //--- ricavo il nome del server APRS-IS cui sono connesso 
      //Serial.println(current_server_aprsis);
    }
    if ( millis() - aprs_login_millis > 60000 ) {                         //--- caduta del network APRS-IS 1 minuto che non ricevo il banner
      Serial.println(F(".. lost connection to APRS-IS server .."));
      aprs_login = false;                                                 //--- dichiaro la connessioner persa
      if ( backupStatusSwitch && !backupStatus ) start_Backup();                //--- se é previsto il backup ma non é attivo il backup attiva il backup
    }
  }
  //-------------------------------------------------- verifica funzionalita server APRS-IS

  //-------------------------------------------------- verifica caduta connessione WiFi e se del caso attiva il backup
  if ( WiFi.status() != WL_CONNECTED && !switch_sleep && String(WiFi1_ssiD) != "") {
    if ( backupStatusSwitch && !backupStatus ) start_Backup();                  
    if ( AP_active ) myIP = ipToString(WiFi.softAPIP());
    else myIP = "0.0.0.0";
    make_display();
    if ( millis() > lost_connection_millis + WiFi_lost_time ) {
      lost_connection_millis = millis();
      WiFi.disconnect();
      delay ( 300 );
      Serial.println(F(".. i try to reconnect to wifi .."));
      if ( String(WiFi2_ssiD) == "" ) WiFi.reconnect();  //--- se non é stata definito il secondo WiFi vai un semplice 'reconnect'
      else {
        setup_WiFi( String(WiFi1_ssiD) , String(WiFi1_pwd) ); //--- ritenta sul primo WiFi
        if ( WiFi.status() != WL_CONNECTED ) {
          setup_WiFi( String(WiFi2_ssiD) , String(WiFi2_pwd) ); //--- se il primo ha fallito ritenta il secondo 
        }
      }
    }    
  }  
  //-------------------------------------------------- verifica caduta connessione WiFi e se del caso attiva il backup

  //-------------------------------------------------- operazioni a seguito ripristino/presenza  connettivita
  if ( WiFi.status() == WL_CONNECTED ) {                                    
    myIP = ipToString(WiFi.localIP());
    if ( !RTC_status ) NTP_query();                   //--- se l'RTC non é settato settalo
    if ( backupStatus ) stop_Backup();                  //--- verifica presenza di un backup in stato attivo, esci da backup e ripristina setup
    if ( !aprs_login ) {                                                    //--- se non c'é un login attivo su APRS-IS ma occorre connettiti a APRS-FI
      if ( igateSwitch || meteo_tx_mode == 2 ) {
        aprsis.stop();
        delay(100);
        Serial.println(F(".. trying to connect to APRS-IS server .."));
        aprsis_setup();
      } 
    }  
  } 
  //-------------------------------------------------- operazioni a seguito ripristino connettivita

  //---------------------------- timer dei beacon e reset tmp_dash_pwd dopo 5 minuti
  if ( millis() - tmp_dash_pwd_millis > 300000 ) {    //--- password dashboard scaduta dopo 30 minuti
    String("admin").toCharArray(tmp_dash_pwd,50); 
    //Serial.print("dash_pwd reset done - new password is: ");Serial.println(tmp_dash_pwd);
    tmp_dash_pwd_millis = millis();
  } 
  //---------------------------- timer dei beacon e reset tmp_dash_pwd dopo 5 minuti

  //---------------------------- refresh dei vari sensori per dashboard 
  if ( millis() - lastSensorUpdate >= sensor_refresh ) {
    token_sensor++;
    if ( token_sensor == 1 ) {
      if ( HTTP_DEBUG_MODE) {
        Serial.print(F("Temp: "));
        Serial.print(TempC);
        Serial.print(F(" | Press: "));
        Serial.println(Press);
      } 
      TempC = getTempC();
      Press = getPressure();
    }
    if ( token_sensor > 1 ) {
      Hum = getHum();
      DewPoint = getDewPoint();
      if ( HTTP_DEBUG_MODE) {
        Serial.print(F("Hum: "));
        Serial.print(Hum);
        Serial.print(F(" | DewPoint: "));
        Serial.println(DewPoint);
      }
      if ( isWSconnected ) updateWebSocket();
      token_sensor=0;
    }
    lastSensorUpdate = millis();
  } 
  //---------------------------- refresh dei vari sensori per dashboard 

  //---------------------------- timer dei beacon
  if ( tx_interval > 9 ) {
    if ( bootCount == 1 && telemSwitch && millis() - lastTlParamBeacon >= (tx_interval*48 * 60000) ) beacon_telemetry_param();  // beacon stringhe parametriche ogni 8 ore
    
    if ( millis() - lastMtBeacon  >= (tx_interval * 60000) ) beacon_meteo();                              //--- beacon meteo                       
    if ( !telemSwitch && millis() - lastStMtBeacon  >= ( tx_interval*2 *  60000 )) beacon_meteo_status(); //--- beacon meteo status ogni 20 minuti se non si usa telemetria per avere nel fumetto i valori di tensione
    if ( telemSwitch && millis() - lastStMtBeacon   >= ( tx_interval*24 * 60000 )) beacon_meteo_status(); //--- beacon meteo status ogni 4 ore se si usa telemetria 

    if ( millis() - lastIgBeacon  >= (tx_interval * 60000) )       beacon_igate();        //--- beacon iGate 
    if ( millis() - lastStIgBeacon >= (tx_interval*24 * 60000) )   beacon_igate_status(); // lo status ogni 4 ore - se iGate riceve segnali resetta pure il contatore

    if ( millis() - pressureTrendTimeout >= (180 * 60000) && BM_sensor_status ) zambrettiRoutine();  // ogni 3 ore zambretti value
  }
  //---------------------------- timer dei beacon

  
  //----------------------------  auto spegnimento di AP o di tutto il modulo WiFi dopo 'Timeout_AP_millis'' se non é configurato un SSiD WiFi
  if ( AP_active && AP_auto_shutdown && millis() - dashboard_activity_millis > Timeout_AP_millis ) {
    Serial.println(F(".. Spengo AP .."));
    WiFi.softAPdisconnect(true);  // spegne l'AP, il client STA resta connesso
    AP_active = LOW;
    delay(350);
    if ( String( WiFi1_ssiD ) == "" ) {
      Serial.println(F(".. Spengo WiFi .."));
      WiFi.mode(WIFI_OFF);  // spengo tutto il modulo WiFi
      myIP = "no WiFi active";
      make_display();
    }
  }
  
  //----------------------------  auto reboot dopo 3 giorni = 72 ore = 259200 secondi 
  cntValue = AUTORESTART_millis - millis()/1000 ;
  if ( cntValue <= 0 ) {
    save_MinMax();
    if ( WiFi.status() == WL_CONNECTED && checkForUpdates() && OTA_enabled ) {
      updateFirmware();    
      OTA_code = 24;
      OTA_logbook();
    }
    if ( autoreboot ) {
      Serial.println(F("now rebooting."));
      display.clearDisplay();
      display.setTextSize(2);
      display.setCursor(10,10);
      display.print("reboot");
      display.setCursor(10,35);
      display.print("now");
      display.display(); 
      OTA_code = 77;
      OTA_logbook();
      ESP.restart();
    }           

  } 
  //----------------------------  auto reboot dopo 3 giorni = 72 ore = 259200 secondi 

  //---------------------------- OTA   
  if ( WiFi.status() == WL_CONNECTED && !token_verify_update ) {
    token_verify_update = true;
    if ( checkForUpdates() && OTA_enabled ) updateFirmware();    
    OTA_logbook();
  }
  //---------------------------- OTA


  //---------------------------- operazioni cicliche ogni 10 secondi 
  if ( millis() > millis_token_tx +10000 ) {  
    millis_token_tx = millis();
    //----------------------------------------------- switcha le informazioni sul display OLED
    retr++;
    if ( retr > 1 ) retr = 0;
    if ( retr == 0 ) make_display();    
    if ( retr == 1 ) {
      if ( DHT22status || BM_sensor_status || AHTstatus ) make_meteo_display();
    }

    weathervane_voltage = 0;
    tlm_voltage=0;                          // tensione 'aux'
    voltage=0;

    ptr = 1;
    while ( ptr <= 10 ) {                                 // esegue 10 letture del sensore
      tmp = analogRead( weathervane_PIN );
      delay(10);
      tmp_Vint = analogRead( VOLTAGE_SENSOR_PIN );
      delay(10);
      tmp_Vext = analogRead( V12_SENSOR_PIN );
      delay(10);
      weathervane_voltage = tmp + weathervane_voltage;
      voltage = tmp_Vint + voltage;
      tlm_voltage= tmp_Vext + tlm_voltage;
      ptr++;
    }

    voltage = voltage/10;                                   // lettura carica batteria con circuito di riduzione interna 2:1 rif: GPIO35
    tlm_voltage = tlm_voltage/160;                          // lettura tensione con circuito di riduzione esterna 4:1 di rif: GPIO39
    voltage_dashboard = ((3.3/4095)*voltage)*2 + drift_battery;   // ATTENZIONE il sulla pista di telemetria é la trasposizione 1/255 del valore voltage_dashboard corretto di drift
    battPercent = 100 * (voltage_dashboard - 3.3) / (4.13 - 3.3);
    if (battPercent > 100) battPercent = 100;
    if (battPercent < 0) battPercent = 0;
   
    weathervane_voltage = weathervane_voltage / 10;         // lettura tensione partitore weathevane rif: GPIO36
    weathervane_voltage = weathervane_voltage + drift_weathervane - 0;  //--- applica drift - il -50 é un drift specifico della mia fixture

    if (  weathervane_voltage > 2075-12 && weathervane_voltage < 2075+12) degrees_weathervane = 0;
    if (  weathervane_voltage > 2808-40 && weathervane_voltage < 2808+40) degrees_weathervane = 45;
    if (  weathervane_voltage > 3798-50 && weathervane_voltage < 3798+50) degrees_weathervane = 90;
    if (  weathervane_voltage > 2330-50 && weathervane_voltage < 2330+50) degrees_weathervane = 135;
    if (  weathervane_voltage > 1830-35 && weathervane_voltage < 1830+35) degrees_weathervane = 180;
    if (  weathervane_voltage > 3582-40 && weathervane_voltage < 3582+40) degrees_weathervane = 225;
    if (  weathervane_voltage > 1919-30 && weathervane_voltage < 1919+30) degrees_weathervane = 270;
    if (  weathervane_voltage > 2110-17 && weathervane_voltage < 2110+17) degrees_weathervane = 315; 

    if ( weathervane_DEBUG_MODE ) {
      Serial.print(weathervane_voltage); Serial.print(" - gradi: " ); Serial.println(degrees_weathervane);
    }

    // --- gestione INA226 ------------------------
    if ( INA226_status ) {     // leggi INA226 solo se non sei in un ciclo beacon
      ina226.readAndClearFlags();
      shuntVoltage_mV = ina226.getShuntVoltage_mV();
      busVoltage_V = ina226.getBusVoltage_V();
      current_mA = ina226.getCurrent_mA();
      if ( A_SCALE ) current_mA = current_mA *10;
      //power_mW = ina226.getBusPower();
      loadVoltage_V  = busVoltage_V + (shuntVoltage_mV / 1000);
      // --- ricalcolo di tlm_voltage perche e installato INA226
      tlm_voltage = int(( loadVoltage_V*255) / voltmeter_param_c);                          //--- parametrizzazione 0-255 per telemetria da 0 a 25.5 Volt
      tlm_current = int(( (current_mA/1000) - amperometer_param_c ) / amperometer_param_b); //--- parametrizzazione 0-255 per telemetria da 0 a 3.200 milliAmpere positivi
    }
    // --- gestione INA226 ------------------------

    token_tx = HIGH;  
    if ( isWSconnected ) updateWebSocket();

  } //----------------------------- operazioni cicliche ogni 8 secondi 

  //--- LETTURA FLUSSI SULLA SERIALE ------------------------------------
  car = Serial.read();
  if (car == 'm' ) {
    while (Serial.read() != '\n') {};
    Tmp="";
    menu();
  }

  if (car == 'd' ) {
   while (Serial.read() != '\n') {};
   status_display();
  }

  if (car == 'r' ) {
    while (Serial.read() != '\n') {};
    righello();
       
    String sTemp = String(TempC,1);
    String sPress = String(Press,1);
    String sHum = String(Hum);

    Serial.print(F("therm: "));
    Serial.print(sTemp);
    Serial.println(F(" C."));
                
    Serial.print(F("hum: "));
    Serial.print(sHum);
    Serial.println(F(" %"));
                 
    Serial.print(F("press SLM: "));Serial.print(sPress);Serial.println(F(" hPA"));
              
    Serial.print(F("TTGO battery: "));Serial.print(voltage_dashboard);Serial.print(F(" - drift: ")); Serial.println(drift_batteryC);

    if (USE_12V && !INA226_status) {
      Serial.print(F("aux voltage: "));Serial.print( tlm_voltage * voltmeter_param_b); Serial.print(F(" V [")); Serial.print(tlm_voltage); Serial.println(F("]"));
    }

    if (INA226_status) {
      Serial.print(F("aux current: "));Serial.print( tlm_current * amperometer_param_b + amperometer_param_c ); Serial.print(F(" A. [")); Serial.print(tlm_current); Serial.println(F("]"));
      Serial.print(F("aux voltage: "));Serial.print( tlm_voltage * voltmeter_param_b                         ); Serial.print(F(" V. [")); Serial.print(tlm_voltage); Serial.println(F("]"));
    } 
    righello();        
  }
   
  if (car == 't' ) {
    while (Serial.read() != '\n') {};
    lastIgBeacon = millis() - int(tx_interval * 60000);
  }

  if (car == 's' ) {
   while (Serial.read() != '\n') {};
   lastMtBeacon = millis() - int(tx_interval * 60000);
  }

/*
  if (car == 'e') {
    while (Serial.read() != '\n') {};
    tmp = 0;
    while (tmp <= 355) {
      Serial.print(tmp); Serial.print(F(" - ")); Serial.println(EEPROM.read(tmp));
      tmp++;    
    } 
  }


  if (car == 'u') {
    while (Serial.read() != '\n') {};
     {
      //IU1FIL-14>APLRT1,WIDE1-1,qAO,IR1CH-11                    :!/81)+PV-kk<&QQRX 145.500 Bat=V (mA)
      //IW1CGW-13>APHRM0,WIDE2-2,qAO,IZ1VCX-10                   :!4419.22N/00748.58E_.../...g...t055r...p...P...h89b10104 IW1CGW weather v.12 [1:0]
      //String rxPacket = "IU1FIL-14>APLRT1,WIDE1-1,qAO,IR1CH-11 :!/81)+PV-kk<&QQRX 145.500 Bat=V (mA)";
      //String rxPacket = "IW1CGW-13>APHRM0,WIDE2-2,qAO,IZ1VCX-10:!4419.22N/00748.58E_.../...g...t055r...p...P...h89b10104 IW1CGW weather v.12 [1:0]";
      //IQ2SW-10>APLRG1,TCPIP*,qAC,T2CHILE                       :!L7H1XPh+#a xGLoRa APRS iGATE-DIGI ARI-Saronno
      //IK2XRO-10>APLRG1,IU2SKJ-10*,qAO,IQ4FE-2                  :!L7ScFPp"pa xGLoRa APRS Batt=4.29V
      //IW1QAF-23>APLRW1,WIDE1-1,qAR,IZ1HKE-10                   :!/8%k8PZxo_ xG252/013g021t061r000p002L000h94b09580Experimental LoRa APRS Wx Station
      //IU1FIL-14>APLRT1,WIDE1-1,qAO,IU1LCU-11                   :!/8/gKPW22k@\Q Bat=V (mA)
    
      String rxPacket = "IU1FIL-14>APLRT1,IR1ZYZ-2*,qAO,IZ5MJO-11:=/8;F=PPNWu !G";
      int pos2 = rxPacket.indexOf(':');
      calc_dist(rxPacket,pos2);
      Serial.print("km: ");Serial.println(Km);
    }
  }


  if (car == '#') {
    while (Serial.read() != '\n') {};
    EEPROM_eraser(0,EEPROM_SIZE);
    Serial.println(F("EEPROM erased"));
  }
  
*/

  if (car == 'x' ) {                        // esce da sleep mode
    while (Serial.read() != '\n') {};
    if ( switch_sleep ) { 
      switch_sleep = LOW;
      Serial.println(F("sleep mode deactivated"));
      EEPROM.write(351, 0); // valore diverso da '127' per sleep_mode disattivo
      EEPROM.commit();   
      delay(30); 
    }  
  }

  if (car == 'y' ) {                        // esce da sleep mode
    while (Serial.read() != '\n') {};
    start_sleep();
    }  

/*

  if (car == 'z') {                     //--- test caduta ad arte server APRS-IS
    while (Serial.read() != '\n') {};
    EEPROM_eraser(0,EEPROM_SIZE);   
    Serial.println(F("ok"));   
  }

  if (car == 'i') {
    while (Serial.read() != '\n') {};
    {
      Serial.print("Shunt Voltage [mV]: "); Serial.println(shuntVoltage_mV);
      Serial.print("Bus Voltage [V]: "); Serial.println(busVoltage_V);
      Serial.print("Load Voltage [V]: "); Serial.println(loadVoltage_V);
      Serial.print("Current[mA]: "); Serial.println(current_mA);
      Serial.print("Bus Power [mW]: "); Serial.println(power_mW);
      if (!ina226.overfLOW) Serial.println("Values OK - no overfLOW");
      else Serial.println("OverfLOW! Choose higher current range");
      Serial.print("Voltage_aux [raw]: "); Serial.println(tlm_voltage);
      Serial.print("Current_aux [raw]: "); Serial.println(tlm_current);
      Serial.println();
    }
  } 

  if (car == 'b') {
    while (Serial.read() != '\n') {};
    NTP_query();
  }

  if (car == 'x') {
    while (Serial.read() != '\n') {};
    zambrettiRoutine();
  }



  */ 

  //--- LETTURA FLUSSI SULLA SERIALE ------------------------------------
  
  

  int packetSize = LoRa.parsePacket();

  if (packetSize) {

    while (LoRa.available()) {
      
      int pos1, pos2, pos3;
      String destCall, digiPath, sourceCall, message, digiPacket,station_symbol,MIC_E_station_symbol;
      
      String rxPacket = LoRa.readString();
      rxPacket = rxPacket.substring(3);

      //Serial.println("RX: " + rxPacket.substring(0,(rxPacket.length()-1)));
      Serial.println("\nRX: " + rxPacket );

      //--- verifica idoneità pacchetto e rilevamento sourceCall, destCall etc
      pos1 = rxPacket.indexOf('>');
      pos2 = rxPacket.indexOf(':');
      digiPath = "";
      sourceCall = rxPacket.substring(0, pos1);
      message = rxPacket.substring(pos2 + 1);
      destCall = rxPacket.substring(pos1 + 1, pos2); 
      pos3 = destCall.indexOf(',');
      if (pos3 > 0) {
        digiPath = destCall.substring(pos3 + 1);
        destCall = destCall.substring(0, pos3);
      }


      //--- verifica idoneità pacchetto
      byte ignore = 10;                                       //--- valore di default '10'
      if (pos1 < 3) ignore = 254;                             //--- stazioni con errore frame '254
      if (pos2 < pos1) ignore = 254;                          //--- stazioni con errore frame '254
      if (destCall == "") ignore = 254;                       //--- stazioni con errore frame '254

      if ( ignore < 254 && pkRx10m <= 255 ) pkRx10m++;        // incrementa il contatore dei pacchetti ricevuti a 10 minuti
   
      //--------------------------- stazioni fake
        if ( rxPacket.indexOf("IU1CGW") > -1 ) ignore = 253; 
        if ( rxPacket.indexOf("AUTES") > -1 ) ignore = 253; 
        if ( rxPacket.indexOf("N0CALL") > -1 ) ignore = 253; 
      //--------------------------- stazioni fake

      //--------------------------- stazioni no ip RF only
        if ( rxPacket.indexOf("NOGATE") > -1 ) ignore = 252; 
        if ( rxPacket.indexOf("RFONLY") > -1 ) ignore = 252; 
        if ( rxPacket.indexOf("TCPIP") > -1 ) ignore = 252; 
        if ( rxPacket.indexOf("TCPXX") > -1 ) ignore = 252; 
      //--------------------------- stazioni no ip RF only


      // ------------------------------------------------ funzioni dell' iGate per codici ignore == 10
      if ( ignore == 10 && igateSwitch ) {
       String igatePacket = rxPacket;
      // if ( igatePacket.indexOf("NOGATE") == -1 && igatePacket.indexOf("RFONLY") == -1 
      //      && igatePacket.indexOf("AUTES") == -1  && igatePacket.indexOf("IU1CGW") == -1 
      //      && igatePacket.indexOf("TCPIP") == -1  && igatePacket.indexOf("TCPXX") == -1 ) {
            igatePacket = igatePacket.substring(0, igatePacket.indexOf(":")) + ",qAO," + String(IGATE_CALLSIGN) + igatePacket.substring(igatePacket.indexOf(":"));
            aprsis_send(igatePacket); 
            //if ( pkig10m <= 255 ) pkig10m++;    // pacchetti igate a 10 minuti
          }
      // } 
      // ------------------------------------------------ funzioni dell' iGate per codici ignore == 10

      
      //-------------- analisi simboli stazioni ascoltate

      station_symbol = rxPacket.substring(pos2+10,pos2+11) + rxPacket.substring(pos2+20,pos2+21);
      MIC_E_station_symbol = rxPacket.substring(pos2+2, pos2+3 ) + rxPacket.substring(pos2+11,pos2+12);

      //--- tabella dei simboli ammessi per il digipeating
      if ( station_symbol == "La" || MIC_E_station_symbol == "La" ) ignore = 251; // LoRa igate icona rossa ( ! gulp ! )
      if ( station_symbol == "L_" || MIC_E_station_symbol == "L_" ) ignore = 251; // LoRa pseudometeo icona bollo blu con simbolo L ( ! gulp ! )
              
      if ( station_symbol == "L#" || MIC_E_station_symbol == "L#" ) ignore = 2; // LoRa DIGI
      if ( station_symbol == "L&" || MIC_E_station_symbol == "L&" ) ignore = 2; // LoRa igate icona nera
      if ( station_symbol == "I#" || MIC_E_station_symbol == "I#" ) ignore = 2; // I-gate equipped digipeater
      if ( station_symbol == "R[" || MIC_E_station_symbol == "R[" ) ignore = 2; // Runner
      if ( station_symbol == "H[" || MIC_E_station_symbol == "H[" ) ignore = 2; // Hiker
      if ( station_symbol == "/[" || MIC_E_station_symbol == "/[" ) ignore = 2; // Human/Person
      if ( station_symbol == "/a" || MIC_E_station_symbol == "/a" ) ignore = 2; // AMBULANCE  
      if ( station_symbol == "/<" || MIC_E_station_symbol == "/<" ) ignore = 2; // Motorcycle 
      if ( station_symbol == "/>" || MIC_E_station_symbol == "/>" ) ignore = 2; // CAR
      if ( station_symbol == "/F" || MIC_E_station_symbol == "/F" ) ignore = 2; // Farm Vehicle (tractor)
      if ( station_symbol == "/O" || MIC_E_station_symbol == "/O" ) ignore = 2; // BALLOON
      if ( station_symbol == "/U" || MIC_E_station_symbol == "/U" ) ignore = 2; // BUS
      if ( station_symbol == "/_" || MIC_E_station_symbol == "/_" ) ignore = 2; // WEATHER Station
      if ( station_symbol == "/b" || MIC_E_station_symbol == "/b" ) ignore = 2; // BIKE
      if ( station_symbol == "/j" || MIC_E_station_symbol == "/j" ) ignore = 2; // JEEP
      if ( station_symbol == "/k" || MIC_E_station_symbol == "/k" ) ignore = 2; // TRUCK
      if ( station_symbol == "/p" || MIC_E_station_symbol == "/p" ) ignore = 2; // ROVER (puppy, or dog)
      if ( station_symbol == "/u" || MIC_E_station_symbol == "/u" ) ignore = 2; // TRUCK (18 wheeler)           
      if ( station_symbol == "/v" || MIC_E_station_symbol == "/v" ) ignore = 2; // VAN   
      if ( station_symbol == "/C" || MIC_E_station_symbol == "/C" ) ignore = 2; // kayak “/C” 
                  
      //--- tabella dei simboli ammessi per il digipeating
  
      

      //--- calcolo distanza da sourceCall [ solo ascolto diretto e pacchetti con georeferenziazione :! ]   
      lastRXstation = "";
      Km = 0;
      if ( rxPacket.indexOf("*") == -1 && sourceCall != "" &&  rxPacket.indexOf(":!") > -1 || rxPacket.indexOf("*") == -1 && sourceCall != "" &&  rxPacket.indexOf(":=") > -1 ) {    
        lastRXstation = sourceCall;
        mem_lastRXstation = sourceCall;
        LogEvent = (rtc.getTime());
        calc_dist(rxPacket,pos2); 
        if ( Km > 0 ) {
          LastKm = Km;
          if ( Km > max_digi_radius ) { 
            ignore = 248;                                  //--- stazione con digipeating negato per radius eccessivo
            //Serial.print(F(" - RX: ")); Serial.print(lastRXstation); Serial.print(F(" - Km: ")); Serial.println(LastKm);
          }
        }

        //--- caricamento MHeard -------------------------------------------------- 
        int shift = 20;
        do { 
          shift--;
          MHeard[shift+1] = MHeard[shift]; 
        } while ( shift >= 0);
              
        MHeard[0] = LogEvent + " " + String(sourceCall) + " SNR " + String(int(LoRa.packetSnr())) + "dB RSSI " + String(LoRa.packetRssi()) + "dBm";
        if ( Km > 0 ) MHeard[0] += " " + String(Km) + "Km";
        //--- caricamento MHeard -------------------------------------------------- 
        
      } 

      // --------------------- regola per stazione che ha un digipeating esclusivo 
      Tmp = String(digi_route); if ( Tmp != "" && sourceCall == Tmp) ignore = 1;
      
      
      // --------------------- regola per consentire digipeating telemetrie e messaggi 
      //if ( rxPacket.indexOf(":T#") != -1  || rxPacket.indexOf(":EQNS.") != -1 || rxPacket.indexOf("UNIT.") != -1 || rxPacket.indexOf("PARM.") != -1 || rxPacket.indexOf("BITS.") != -1 ) ignore = 3;
      if ( rxPacket.indexOf(":") != -1 || rxPacket.indexOf("?") != -1 || rxPacket.indexOf("<") != -1 || rxPacket.indexOf("T#") != -1 ) ignore = 3;
 

      // --------------------- regola per stazione che ha un digipeating negato 
      if ( String(digi_banned).equals(sourceCall) ) ignore = 249;   

      // --------------------- regola per stazione già ripetuta una volta  
      if ( rxPacket.indexOf("*") > 1 ) ignore = 250; 
        


      if ( debug_digi ) { 
          Serial.print(F(".. ignore code: "));  Serial.print(ignore);  Serial.print(F(" "));
          if ( ignore == 1 ) Serial.println(F("** OK station in exclusive digipeater route"));  
          if ( ignore == 2 ) Serial.println(F("** OK station symbol ok in digipeater route")); 
          if ( ignore == 3 ) Serial.println(F("** OK telemetrix & msg in digipeater route"));  
          if ( ignore == 4 ) Serial.println(F("** OK query message"));  
          //================================================================
          if ( ignore == 10 ) Serial.println(F("** data not managed in digipeating")); 
          if ( ignore == 254 ) Serial.println(F("** bad packet 'destcall' not found"));
          if ( ignore == 253 ) Serial.println(F("** station fake"));
          if ( ignore == 252 ) Serial.println(F("** station no via ip only RF but not digipeat")); 
          if ( ignore == 251 ) Serial.println(F("** bad packet not compliance aprs.org"));
          if ( ignore == 250 ) Serial.println(F("** station already repeated"));
          if ( ignore == 249 ) Serial.println(F("** station banned for digipeating")); 
          if ( ignore == 248 ) { 
            Serial.print(F("** station @ "));Serial.print(LastKm);
            Serial.print(F( "Km, over max digipeater radius @ "));
          } 
      }
  
        // -------------------------------------------- funzioni del digipeater per i soli codici ignore minori di 10
        if ( digiSwitch && ignore < 10 ) {
          if (int callIndex = digiPath.indexOf(String(IGATE_CALLSIGN)) > -1 && digiPath.indexOf(String(IGATE_CALLSIGN) + "*") == -1) {
            digiPath = digiPath.substring(0, callIndex - 1) + digiPath.substring(callIndex + String(IGATE_CALLSIGN).length());
          }
          if (int paradigmIndex = digiPath.indexOf("WIDE1-") > -1 && digiSwitch && digiPath.indexOf(String(IGATE_CALLSIGN) + "*") == -1 && rxPacket.indexOf(String(METEO_CALLSIGN)) == -1 && sourceCall.indexOf(String(IGATE_CALLSIGN)) == -1) {
            paradigmIndex = digiPath.indexOf("WIDE1-");
            if (paradigmIndex == 0) paradigmIndex = 1;
            if (paradigmIndex == 1) digiPath = digiPath.substring(0, paradigmIndex - 1) + "," + String(IGATE_CALLSIGN) + "*,WIDE1*" + digiPath.substring(paradigmIndex + 6);
            else digiPath = digiPath.substring(0, paradigmIndex - 1) + "," + String(IGATE_CALLSIGN) + "*,WIDE1*" + digiPath.substring(paradigmIndex + 7);
            if (digiPath.indexOf(",") != 0) digiPath = "," + digiPath;
            digiPacket = sourceCall + ">" + destCall + digiPath + ":" + message;
            delay(750);             //--- 20/02/2025
            lora_send(digiPacket);
            if ( pkTx10m <= 255 ) pkTx10m++;    // pacchetti digipeater a 10 minuti

          } else if (digiSwitch && DIGI_IGNORE_PARADIGM && digiPath.indexOf("*") == -1 && (millis() > lastDigipeat + 30000 || lastDigipeat == 0) && digiPath.indexOf(String(IGATE_CALLSIGN) + "*") == -1 && rxPacket.indexOf(String(METEO_CALLSIGN)) == -1 && sourceCall.indexOf(String(IGATE_CALLSIGN)) == -1) {
            //else if (digiSwitch && DIGI_IGNORE_PARADIGM && digiPath.indexOf("*") == -1 && (millis() > lastDigipeat + 600000 || lastDigipeat == 0) && digiPath.indexOf(String(IGATE_CALLSIGN) + "*") == -1 && rxPacket.indexOf(String(METEO_CALLSIGN)) == -1 && sourceCall.indexOf(String(IGATE_CALLSIGN)) == -1) {
            //else if (digiSwitch && DIGI_IGNORE_PARADIGM && digiPath.indexOf("*") == -1 && (millis() > lastDigipeat + 600000 || lastDigipeat == 0)                                                                                                                                                              ) {
            
            lastDigipeat = millis();
            digiPath = digiPath + "," + String(IGATE_CALLSIGN) + "*";
            if (digiPath.indexOf(",") != 0) {
              digiPath = "," + digiPath;
              Serial.print("digiPath.indexOf(',') + '*' digiPath: ");Serial.println(digiPath);
            }
            digiPacket = sourceCall + ">" + destCall + digiPath + ":" + message;
            delay(750);             //--- 20/02/2025
            lora_send(digiPacket);
            if ( pkTx10m <= 255 ) pkTx10m++;    // pacchetti digipeater a 10 minuti
          } else if (digiSwitch && DIGI_IGNORE_PARADIGM) {
              Serial.println("Station not repeated.");
            }
        } //--- end modulo digipeater 

      if ( USE_LASTRX_STATUS &&  ignore <= 10 && lastRXstation != "" && tx_interval > 1 && igateSwitch ) {
        String statusMessage = String(IGATE_CALLSIGN) + ">APLHI0:>Last RX: " + String(lastRXstation) + " SNR=" + String(LoRa.packetSnr()) + "dB RSSI=" + String(LoRa.packetRssi()) + "dBm";
        if ( Km > 0 ) statusMessage = statusMessage + " - " + String(Km) + "Km";
        aprsis_send(statusMessage);
        lastStIgBeacon=millis();    // per 3 ore non verrà inviato lo status 'istituzionale' cosi rimane l'ultima stazione ricevuta.
      }

    } // --- end pacchetto valido con preambolo: \x3c\xff\x01
  } // --- end received - packetSize == true
 
//if (millis()- millis_token_rx > 8000 ) radio.startReceive();// Rimette il modulo in modalità ricezione per il prossimo messaggio
//--- svuota buffer WiFiClient aprsis

while ( igateSwitch && aprsis.available() || meteo_tx_mode == 2 && aprsis.available() ) {
  String apstring;
  char aprx = aprsis.read();
  apstring += aprx;
  if (aprx == '\n') {
    //Serial.print("apstring: ");Serial.println(apstring);
    if (apstring.indexOf("logresp") == -1) {
      // incoming packet handling
      //Serial.print("incoming packet handling: ");Serial.println(apstring);
    }
    apstring = "";
  }
}

if ( count >0 ) USE_anemometer = HIGH;      // rilevato un conteggio quindi esiste un anemometro

if ( USE_anemometer ) {

  if (windActualSpeed > gust) {
      gust = windActualSpeed;
      windLastGust = millis();
  }

  if (windTimeout + (ANEMO_RECALC_LIMIT_TIMEOUT * 1000) < millis()) {
    //windLongPeriodSpeed = float(1 / ((millis() - windTimeout) / (windMeterSpinsInTimeout * ANEMOMETER_LENGTH * 1000)));
    windLongPeriodSpeed = float(1 / ((millis() - windTimeout) / (windMeterSpinsInTimeout * anemometer_lenght / 100 * 1000)));
    //windLongPeriodSpeed = float(windMeterSpinsInTimeout * 1000 / (millis() - windTimeout)) * float(ANEMOMETER_LENGTH);;
    windTimeout = millis();
    Serial.println("Long-period wind: " + String(windLongPeriodSpeed) + " m/s || gust: " + String(gust) + " m/s");
    windMeterSpinsInTimeout = 0;
  }

  if (millis() > windCycleDuration + (ANEMO_RECALC_ACTUAL_SPEED * 1000)) windActualSpeed = 0;
  if (millis() > windLastGust + (ANEMO_RECALC_LIMIT_TIMEOUT * 1000)) gust = windActualSpeed;

}


  //------------------------------------------------ server dashboard
  //------------------------------------------------ server dashboard
  ws.cleanupClients();
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
                  
              if (BMEstatus || BMPstatus && AHTstatus || DHT22status && BMPstatus)  client.println(tempToWeb(TempC) +  DewPointToWeb(DewPoint) + HumToWeb(int(Hum)) + pressToWeb(Press) );
              if (BMPstatus && !AHTstatus && !DHT22status)                          client.println(tempToWeb(TempC)                                                 + pressToWeb(Press) );
              if (DHT22status && !AHTstatus && !BMPstatus)                          client.println(tempToWeb(TempC) +  DewPointToWeb(DewPoint) + HumToWeb(int(Hum)) );
              
              if (GETIndex(header, "/api/graphs-json"))

              if (BMEstatus || BMPstatus && AHTstatus || DHT22status && BMPstatus) client.println("{\"temperature\": [" + String(tempValues) + "], \"pressure\": [" + String(pressValues) + "], \"Hum\": [" + String(HumValues) + "] , \"Wind\": [" + String(windValues) + "] }");
              if (BMPstatus && !AHTstatus && !DHT22status)                         client.println("{\"temperature\": [" + String(tempValues) + "], \"pressure\": [" + String(pressValues) + "],                                        \"Wind\": [" + String(windValues) + "] }");
              if (DHT22status && !AHTstatus && !BMPstatus)                         client.println("{\"temperature\": [" + String(tempValues) + "],                                            , \"Hum\": [" + String(HumValues) + "] , \"Wind\": [" + String(windValues) + "] }");

              //if (GETIndex(header, "/api/json"))
              //  client.println("{\"general\": {\"version\":\"" + String(Control) + "\", \"system_time\":" + String(millis()) + ", \"voltage\":" + String(voltage) + ", \"battery\":" + String(battPercent) + ", \"wifi_status\":" + (check_wifi() ? "true" : "false") + ", \"wifi_signal_db\":" + (check_wifi() ? String(WiFi.RSSI()) : "0") + ", \"WiFi1_ssiD\":\"" + String(WiFi.SSID()) + "\", \"wifi_hostname\":\"" + String(Hostname) + "\", \"bmp280_status\":" + (getBM_sensor_status() ? "true" : "false") + "}, \"lora\": {\"METEO_CALLSIGN\":\"" + String(METEO_CALLSIGN) + "\", \"digi_enabled\":" + (digiSwitch ? "true" : "false") + "\", \"igate_callsign\":\"" + String(IGATE_CALLSIGN) + ", \"aprs_is_status\":" + (check_aprsis() ? "true" : "false") + ", \"aprs_server\":\"" + (check_aprsis() ? String(APRSISServer) : "disconnected") + ", \"last_rx\":\"" + String(lastRXstation) + "\"" + "}, \"meteo\": {\"temperature\":" + valueForJSON(tempToWeb(getTempC())) + ", \"pressure\":" + valueForJSON(pressToWeb(getPressure()))  + ", \"Hum\":" + valueForJSON(HumToWeb(getHum())) + ", \"min_temperature\":" + (getBM_sensor_status() ? String(minTemp) : "0") + ", \"max_temperature\":" + (getBM_sensor_status() ? String(maxTemp) : "0") + ", \"min_pressure\":" + (getBM_sensor_status() ? String(minPress) : "0") + ", \"max_pressure\":" + (getBM_sensor_status() ? String(maxPress) : "0"));
              } else {
                  client.println(String(webPageStart));

                  if (GETIndex(header, "/switch-meteo")) {
                    if ( !check_pwd() ) client.print(loginReload);
                    else {
                      meteo_tx_mode++;
                      
                      
                      if ( meteo_tx_mode > 2 ) meteo_tx_mode=0; // dalla 20240927 la modalità di invio rf+ip non é più possibile per la stringa meteo
                      if ( meteo_tx_mode == 2 ) aprs_login = LOW;     // richiedi nuova connessione a APRS-IS
                      EEPROM.write(164,meteo_tx_mode);
                      EEPROM.commit();
                      delay(30);
                      client.println(webReload);
                    }  
                  }

                  if (GETIndex(header, "/switch_sleep")) {
                    if ( !check_pwd() ) client.print(loginReload);
                    else {
                      switch_sleep = !switch_sleep;
                      verifica_parametri();   // serve per eventualmente spegnere igate e digipeater o non impostarlo se tx_param é a '0' 
                      if ( switch_sleep ) EEPROM.write(351, 212); // valore '127' per sleep_mode attivo
                      if ( !switch_sleep ) EEPROM.write(351, 0); // valore diverso da '127' per sleep_mode disattivo
                      if ( switch_sleep ) EEPROM.write(165, 0); // switch backup disattivato locazione 165
                      EEPROM.commit();   
                      delay(30);     
                      client.println(webReload);  // gli on/off richiedono il caricamento della pagina
                    }
                  }


                  if (GETIndex(header, "/switch-digi")) {
   
                    if ( !check_pwd() ) client.print(loginReload);
                    else {
                      digiSwitch = !digiSwitch;
                      if ( digiSwitch ) {
                        igateSwitch = LOW;
                        EEPROM.write(166,0);
                      }  
                      EEPROM.write(167,digiSwitch);
                      EEPROM.commit();
                      delay(30);
                      client.println(webReload);
                    } 
                  }


                  if (GETIndex(header, "/switch-igate")) {
                    if ( !check_pwd() ) client.print(loginReload);
                    else {
                      igateSwitch = !igateSwitch;
                      if ( igateSwitch ) {
                        digiSwitch = LOW;
                        EEPROM.write(167,0);  //--- digipeater
                        EEPROM.write(166,1);  //--- iGate
                        aprs_login = LOW; // richiedi nuova connessione a APRS-IS
                      } 
                      EEPROM.write(166,igateSwitch); 
                      EEPROM.commit();
                      delay(30);
                      client.println(webReload);
                    }  
                  }
                  

                  if (GETIndex(header, "/switch-zambretti")) {
                    if ( !check_pwd() ) client.print(loginReload);
                    else {
                      zambretti = !zambretti;
                      if ( zambretti ) EEPROM.write(52,1);
                      else EEPROM.write(52,0);
                      EEPROM.commit();
                      delay(30);
                      client.println(setupReload);  // gli on/off richiedono il caricamanto della pagina
                    }  
                  }

                  if (GETIndex(header, "/switch-telem")) {
 
                    if ( !check_pwd() ) client.print(loginReload);
                    else {
                      telemSwitch = !telemSwitch;
                      if (telemSwitch ) {
                        EEPROM.write(304,1); 
                        cnt_param=0;
                        cnt_telem=0;
                        lastTlParamBeacon = millis() - int(tx_interval*48 * 60000);  //--- invio immediato stringhe parametriche 
                        lastMtBeacon = millis();  //--- blocca invio beacon meteo
                      }
                      if (!telemSwitch ) EEPROM.write(304,0);
                      EEPROM.commit();
                      delay(30);
                      client.println(webReload);
                    }  
                  }


                  if (GETIndex(header, "/switch-wunder")) {
                    if ( !check_pwd() ) client.print(loginReload);
                    else {
                      wunderSwitch = !wunderSwitch;
                      if (wunderSwitch == true ) EEPROM.write(276,1);
                      if (wunderSwitch == false) EEPROM.write(276,0);
                      EEPROM.commit();
                      delay(30);
                      wunderSendStatus=0;
                      client.println(webReload);
                    }  
                  }


                  if (GETIndex(header, "/update")) {

                    if ( !check_pwd() ) client.print(loginReload);
                    else {
                      client.println("<br><a>.. check for update ..</a>");
                      if ( checkForUpdates() ) {
                        client.println("<br><a>.. an update is available ..</a>");
                        if ( OTA_enabled ) {
                          client.println("<br><a>.. installing the update ..</a>");
                          client.println("<br><a>.. please, close this tab ..</a>");
                          updateFirmware();
                        }
                        else {
                          OTA_code = 98;
                          OTA_logbook(); 
                          client.println(webReload);
                        }
                      } 
                      else {
                        client.println("<br><a>.. no update is available ..</a>");
                        OTA_code = 99;
                        OTA_logbook(); 
                        client.println(webReload);
                      }
                    }
                  }
          
           
                  if (GETIndex(header, "/restart")) {

                    if ( !check_pwd() ) client.print(loginReload);
                      else {
                        save_MinMax();
                        client.println("<br><a>.. restart in progress ..</a>");
                        delay(1000);
                        client.println("<br><a>.. please, close this tab ..</a>");
                        delay(1000);
                        ESP.restart();
                      }
                  }

            
                  if (GETIndex(header, "/beacon")) {

                    if ( !check_pwd() ) client.print(loginReload);
                    else {
                      if ( meteo_tx_mode > 0 || digiSwitch || igateSwitch || wunderSwitch ) {
                        lastMtBeacon = millis() - int(tx_interval * 60000);
                        lastStMtBeacon = millis() - int(tx_interval*24 * 60000);
                        lastIgBeacon = millis() - int(tx_interval * 60000);
                        lastStIgBeacon = millis() - int(tx_interval*24 * 60000);
                        client.println("<br>.. sending beacons ..<br>");
                      }
                      else client.println("<br>.. all systems are off, no beacons sent ..<br>");
                      delay(2000);
                      client.println(webReload);
                    } 
                  }
                

                  if (GETIndex(header, "/reset-temp")) {

                    if ( !check_pwd() ) client.print(loginReload);
                    else {
                      minTemp = TempC;
                      maxTemp = minTemp;
                      client.println("<br>Temperature reset done.<br>");
                      save_MinMax();
                      delay(1000);
                      client.println(webReload);
                    } 
                  }


                  if (GETIndex(header, "/reset-hum")) {

                    if ( !check_pwd() ) client.print(loginReload);
                    else {
                      minHum = Hum;
                      maxHum = minHum;
                      client.println("<br>Humidity reset done.<br>");
                      delay(1000);
                      client.println(webReload);
                    }  
                  }


                  if (GETIndex(header, "/reset-press")) {

                    if ( !check_pwd() ) client.print(loginReload);
                    else {
                      minPress = Press;
                      maxPress = minPress;
                      client.println("<br>Pressure reset done.<br>");
                      save_MinMax();
                      delay(1000);
                      client.println(webReload);
                    }
                  }

                  if (GETIndex(header, "/set_mod")) {

                    if ( !check_pwd() ) client.print(loginReload);
                    else {
                      if ( mod_type != 212 ) mod_type = 212;
                      else mod_type = 0;
                      EEPROM.write(348,mod_type);
                      EEPROM.commit();
                      delay(30);
                      lora_setup();
                      client.println(setupReload);
                    }
                  }

                  if (GETIndex(header, "/power")) {
                    if ( !check_pwd() ) client.print(loginReload);
                    Tmp="power";
                    client.println(web_ChangePrompt_power);
                  }

                  //if (GETIndex(header, "/rx_gain")) {
                  //  Tmp="rx_gain";
                  //  client.println(web_ChangePrompt_rx_gain);
                  //}

                  if (GETIndex(header, "/login")) {
                    Tmp="login";
                    client.println(web_ChangePrompt_pwd);
                  }

                  if (GETIndex(header, "/frequency")) {
                    if ( !check_pwd() ) client.print(loginReload);
                    Tmp="frequency";
                    client.println(web_ChangePrompt_frequency);
                  }


                  if (GETIndex(header, "/backup_igate")) {
                    if ( !check_pwd() ) client.print(loginReload);
                    else {
                      backupStatusSwitch = !backupStatusSwitch;
                      if (backupStatusSwitch == true ) EEPROM.write(165,1);
                      if (backupStatusSwitch == false) EEPROM.write(165,0);
                      EEPROM.commit();
                      delay(30);
                      make_display();
                      client.println(setupReload);  // gli on/off richiedonoil carticamento della pagina
                    }  
                  }


                  if (GETIndex(header, "/iGateBeaconOnRF")) {
                    if ( !check_pwd() ) client.print(loginReload);
                    else {
                      iGateBeaconOnRF = !iGateBeaconOnRF;
                      EEPROM.write(277,iGateBeaconOnRF);
                      EEPROM.commit();
                      delay(30);
                      client.println(setupReload);  // gli on/off richiedono il caricamanto della pagina
                    }  
                  }


                  if (GETIndex(header, "/ota_enabled_set")) {
                    if ( !check_pwd() ) client.print(loginReload);
                    else {
                      OTA_enabled = !OTA_enabled;
                      if ( !OTA_enabled ) EEPROM.write(338,212);  // set '212' for disable function
                      else EEPROM.write(338,1);  // set '1' for enable function
                      EEPROM.commit();
                      delay(30);
                      client.println(setupReload);  // gli on/off richiedono il caricamanto della pagina
                    }  
                  }


                  if (GETIndex(header, "/use_12v")) {
                    if ( !check_pwd() ) client.print(loginReload);
                    else {
                      USE_12V = !USE_12V;
                      EEPROM.write(305,USE_12V);
                      EEPROM.commit();
                      delay(30);
                      client.println(setupReload);  // gli on/off richiedono il caricamanto della pagina
                    } 
                  }

                
                  if (GETIndex(header, "/hide_aprs_server")) {
                    if ( !check_pwd() ) client.print(loginReload);
                    else {
                      hide_aprs_server = !hide_aprs_server;
                      if ( hide_aprs_server ) EEPROM.write(349,212);
                      else EEPROM.write(349,0);
                      EEPROM.commit();
                      delay(30);
                      aprs_login = LOW; // richiedi nuova connessione a APRS-IS
                      client.println(setupReload);  // gli on/off richiedono il caricamanto della pagina
                    } 
                  }



                  if (GETIndex(header, "/auto_reboot")) {
                    if ( !check_pwd() ) client.print(loginReload);
                    else {
                      autoreboot = !autoreboot;
                      if ( autoreboot) EEPROM.write(352, 1); // valore '1' per autoreboot attivo
                      if ( !autoreboot ) EEPROM.write(352, 0); // valore '0' per autoreboot disattivo
                      EEPROM.commit();   
                      delay(30);     
                      client.println(setupReload);  // gli on/off richiedono il caricamento della pagina
                    }
                  }

                  
                  if (GETIndex(header, "/AP_auto_shutdown")) {
                    if ( !check_pwd() ) client.print(loginReload);
                    else {
                      AP_auto_shutdown = !AP_auto_shutdown;
                      if ( AP_auto_shutdown) EEPROM.write(353, 1); // valore '1' per AP_auto_shutdown attivo
                      if ( !AP_auto_shutdown ) EEPROM.write(353, 0); // valore '0' per AP_auto_shutdown disattivo
                      EEPROM.commit();   
                      delay(30);     
                      client.println(setupReload);  // gli on/off richiedono il caricamento della pagina
                    }
                  }
             

                  if (GETIndex(header, "/a_scale")) {
                    if ( !check_pwd() ) client.print(loginReload);
                    else {
                      A_SCALE = !A_SCALE;
                      EEPROM.write(337,A_SCALE);
                      EEPROM.commit();
                      delay(30);
                      client.println(setupReload);  // gli on/off richiedono il carticamanto della pagina
                    } 
                  }

                  if (GETIndex(header, "/oled")) {
                    if ( !check_pwd() ) client.print(loginReload);
                    else {
                      USE_oled = !USE_oled;
                      display.dim(!USE_oled );
                      display.display();
                      EEPROM.write(168, !USE_oled); // valore '0' per OLED acceso 
                      EEPROM.commit();   
                      delay(30);           
                      client.println(setupReload);  // gli on/off richiedono il caricamanto della pagina
                    }
                  }
                  


                  if (GETIndex(header, "/set_beacon_interval")) {
                    if ( !check_pwd() ) client.print(loginReload);
                    Tmp="beacon_interval";
                    client.println(web_ChangePrompt_beacon_interval);
                  }


                  if (GETIndex(header, "/vw_drift")) {
                    if ( !check_pwd() ) client.print(loginReload);
                    Tmp="vw_drift";
                    client.println(web_ChangePrompt_vw);
                  }


                  if (GETIndex(header, "/igate_ssid")) {
                    if ( !check_pwd() ) client.print(loginReload);
                    Tmp="igate_ssid";
                    client.println(web_ChangePrompt_SSid);
                  }


                  if (GETIndex(header, "/meteo_ssid")) {
                    if ( !check_pwd() ) client.print(loginReload);
                    Tmp="meteo_ssid";
                    client.println(web_ChangePrompt_SSid);
                  }


                  if (GETIndex(header, "/circumference")) {
                    if ( !check_pwd() ) client.print(loginReload);
                    Tmp="circumference";
                    client.println(web_ChangePrompt_anemo);
                  }


                  if (GETIndex(header, "/radius")) {
                    if ( !check_pwd() ) client.print(loginReload);
                    Tmp="radius";
                    client.println(web_ChangePrompt_radius);
                  }


                  if (GETIndex(header, "/route")) {
                    if ( !check_pwd() ) client.print(loginReload);
                    Tmp="route";
                    client.println(web_ChangePrompt_route);
                  }


                  if (GETIndex(header, "/banned")) {
                    if ( !check_pwd() ) client.print(loginReload);
                    Tmp="banned";
                    client.println(web_ChangePrompt_banned);
                  }


                  if (GETIndex(header, "/change-drift_temp")) {
                    if ( !check_pwd() ) client.print(loginReload);
                    Tmp="drift_temp";
                    client.println(web_ChangePrompt_temp);
                  }


                  if (GETIndex(header, "/change-drift_pres")) {
                    if ( !check_pwd() ) client.print(loginReload);
                    Tmp="drift_pres";
                    client.println(web_ChangePrompt_pres);
                  }


                  if (GETIndex(header, "/change-meteo_info")) {
                    if ( !check_pwd() ) client.print(loginReload);
                    Tmp="meteo_info";
                    client.println(web_ChangePrompt_text);
                  }


                  if (GETIndex(header, "/change-igate_info")) {
                    if ( !check_pwd() ) client.print(loginReload);
                    Tmp="igate_info";
                    client.println(web_ChangePrompt_text);
                  }


                  if (GETIndex(header, "/WiFi1_ssiD")) {
                    if ( !check_pwd() ) client.print(loginReload);
                    Tmp="WiFi1_ssiD";
                    client.println(web_ChangePrompt_text);
                  }


                  if (GETIndex(header, "/WiFi1_pwd")) {
                    if ( !check_pwd() ) client.print(loginReload);
                    Tmp="WiFi1_pwd";
                    client.println(web_ChangePrompt_pwd);
                  }


                  if (GETIndex(header, "/WiFi2_ssiD")) {
                    if ( !check_pwd() ) client.print(loginReload);
                    Tmp="WiFi2_ssiD";
                    client.println(web_ChangePrompt_text);
                  }


                  if (GETIndex(header, "/WiFi2_pwd")) {
                    if ( !check_pwd() ) client.print(loginReload);
                    Tmp="WiFi2_pwd";
                    client.println(web_ChangePrompt_pwd);
                  }


                  if (GETIndex(header, "/WiFi_AP_pwd")) {
                    if ( !check_pwd() ) client.print(loginReload);
                    Tmp="WiFi_AP_pwd";
                    client.println(web_ChangePrompt_pwd);
                  }    


                  if (GETIndex(header, "/dash_pwd")) {
                    if ( !check_pwd() ) client.print(loginReload);
                    Tmp="dash_pwd";
                    client.println(web_ChangePrompt_pwd);
                  }               


                  if (GETIndex(header, "/callsign")) {
                    if ( !check_pwd() ) client.print(loginReload);
                    Tmp="callsign";
                    client.println(web_ChangePrompt_text);
                  }


                  if (GETIndex(header, "/passcode")) {
                    if ( !check_pwd() ) client.print(loginReload);
                    Tmp="passcode";
                    client.println(web_ChangePrompt_value);
                  }


                  if (GETIndex(header, "/WU_iD")) {
                    if ( !check_pwd() ) client.print(loginReload);
                    Tmp="WU_iD";
                    client.println(web_ChangePrompt_text);
                  }
             
             
                  if (GETIndex(header, "/WU_pwd")) {
                    if ( !check_pwd() ) client.print(loginReload);
                    Tmp="WU_pwd";
                    client.println(web_ChangePrompt_pwd);
                  }
             
             
                  if (GETIndex(header, "/drift_batteryC")) {
                    if ( !check_pwd() ) client.print(loginReload);
                    Tmp="drift_batteryC";
                    client.println(web_ChangePrompt_value);
                  }


                  if (GETIndex(header, "/altitude")) {
                    if ( !check_pwd() ) client.print(loginReload);
                    Tmp="altitude";
                    client.println(web_ChangePrompt_value);
                  }


                  if (GETIndex(header, "/GMT")) {
                    if ( !check_pwd() ) client.print(loginReload);
                    Tmp="GMT";
                    client.println(web_ChangePrompt_value);
                  }


                  if (GETIndex(header, "/meteo_latlong")) {
                    if ( !check_pwd() ) client.print(loginReload);
                    Tmp="meteo_latlong";
                    client.println(web_ChangePrompt_latlong);
                  }

                  if (GETIndex(header, "/aprs_server")) {
                    if ( !check_pwd() ) client.print(loginReload);
                    Tmp="aprs_server";
                    client.println(web_ChangePrompt_value);
                  }

                  if (GETIndex(header, "/igate_latlong")) {
                    if ( !check_pwd() ) client.print(loginReload);
                    Tmp="igate_latlong";
                    client.println(web_ChangePrompt_latlong);
                  }


                  if (GETIndex(header, "/new-value")) {
                    apktIndex = header.indexOf("GET /new-value");
                    String newData = header.substring(apktIndex + 15, header.indexOf("HTTP/") - 1);
                    if (newData != "null" ) {
                      newData.replace("%20", " ");  // rimpiazza eventuali %20 con spazi
                      if ( Tmp == "meteo_latlong") newData.replace(" ", "");  // elimina gli spazi
                      if ( Tmp == "igate_latlong") newData.replace(" ", "");  // elimina gli spazi
                      if ( Tmp == "callsign") newData.replace(" ", "");  // elimina gli spazi
                      if ( Tmp == "callsign") newData.toUpperCase();          // callsign tutto maiuscolo
                      
                      //if (Tmp == "callsign") newData.replace("%20", " ");  // converti in maiuscolo
                      ptr = newData.length();       // lunghezza della stringa
                      sep = newData.indexOf(',');   // posizione del separatore lat / long
                      if ( ptr > 49 ) ptr = 49;
                      
                      if (Tmp == "igate_ssid" ) {
                        newData.toCharArray(tmp_buffer,3);  
                        //Serial.print("valore di tmp_buffer: ");Serial.println(tmp_buffer);
                        Tmp = String(tmp_buffer);
                        if ( Tmp == "")  igate_ssiD = 100; 
                        else igate_ssiD = atoi(tmp_buffer); 
                        //Serial.print("valore di igate_ssiD: ");Serial.println(igate_ssiD);
                        verifica_parametri();        
                        EEPROM.write(13, (igate_ssiD));
                        EEPROM.commit();      
                        delay(30); 
                        verifica_parametri(); // verifica se correggere IGATE_CALLSIGN
                      }  
       
                      if (Tmp == "meteo_ssid" ) {
                        newData.toCharArray(tmp_buffer,3);   
                        meteo_ssiD = atoi(tmp_buffer); 
                        verifica_parametri();        
                        EEPROM.write(12, (meteo_ssiD));
                        EEPROM.commit();      
                        delay(30); 
                      } 

                      if (Tmp == "circumference" ) {
                        newData.toCharArray(anemometer_lenghtC,50);  
                        verifica_parametri(); 
                        EEPROM_writer(306,306+ptr-1,anemometer_lenghtC);
                        EEPROM_eraser(306+ptr,310);
                      } 

                      if (Tmp == "frequency" ) {
                        newData.toCharArray(frequencyC,50); 
                        lora_setup(); 
                        EEPROM_writer(41,46,frequencyC);
                      } 

                      if (Tmp == "login" ) {
                        newData.toCharArray(tmp_dash_pwd,9);  
                        tmp_dash_pwd_millis = millis(); 
                        //client.println(setupReload);
                      } 

                      if (Tmp == "igate_info" ) {
                          newData.toCharArray(igate_info,50);           // da string a char
                          verifica_parametri();
                          EEPROM_writer(112,112+ptr-1,igate_info);
                          EEPROM_eraser(112+ptr,152);
                      }    

                      if (Tmp == "meteo_info" ) {
                        newData.toCharArray(meteo_info,50);
                        EEPROM_writer(60,60+ptr-1,meteo_info);
                        EEPROM_eraser(60+ptr,101);
                      }   

                      if (Tmp == "drift_temp" ) {
                        if (ptr > 5 ) ptr = 5;
                        char tmp_drift_thermC[50];
                        newData.toCharArray(tmp_drift_thermC,50);   
                        if (atof(tmp_drift_thermC) <= 99 && atof(tmp_drift_thermC) >= -99 ) {
                          newData.toCharArray(drift_thermC,50);
                          EEPROM_writer(47,47+ptr-1,drift_thermC);                   // salvare nella eeprom il varore in Char
                          EEPROM_eraser(47+ptr,51);
                          drift_therm = atof(drift_thermC);                          // valore matematico
                        }
                      }   
                
                      if (Tmp == "drift_pres" ) {
                        if (ptr > 5 ) ptr = 5;
                        newData.toCharArray(tmp_buffer,50);   
                        if (atoi(tmp_buffer) >=-10 && atoi(tmp_buffer) <= 10 ) {
                            drift_pres = atoi(tmp_buffer);                            // valore matematico
                            EEPROM.write(169, (drift_pres+10));
                            EEPROM.commit();   
                            delay(30);                       
                        }   
                      }  

                      if (Tmp == "drift_batteryC" ) {
                        newData.toCharArray(drift_batteryC,50);   
                        verifica_parametri();                           // verifica e riporcava valore numerico float della char
                        EEPROM_writer(311,311+ptr-1,drift_batteryC);
                        EEPROM_eraser(311+ptr,315);                      
                      }   
                  
                      if (Tmp == "altitude" ) {
                        newData.toCharArray(altitude,50);   
                        EEPROM_writer(35,35+ptr-1,altitude);
                        EEPROM_eraser(35+ptr,38);                    
                      }   

                      if (Tmp == "vw_drift" ) {
                        newData.toCharArray(tmp_buffer,50);
                        drift_weathervane = atoi(tmp_buffer);
                        verifica_parametri;   
                        EEPROM.write(316, (drift_weathervane+125));
                        EEPROM.commit();   
                        delay(30);                       
                        }   
                
                      if (Tmp == "power" ) {
                        newData.toCharArray(tmp_buffer,50); 
                        LoRa_power = atoi(tmp_buffer); 
                        verifica_parametri();  
                        //LoRa.setTxPower(LoRa_power);
                        delay(1000);
                        EEPROM.write(39, (LoRa_power));
                        EEPROM.commit();      
                        delay(30);                    
                      }  

                      /*
                      if (Tmp == "rx_gain" ) {
                        newData.toCharArray(tmp_buffer,50); 
                        LoRa_rx_gain = atoi(tmp_buffer); 
                        verifica_parametri();  
                        //LoRa.setGain(LoRa_rx_gain);
                        delay(1000);
                        EEPROM.write(337, (LoRa_rx_gain));
                        EEPROM.commit();      
                        delay(30);                    
                      }
                      */
 
                      if (Tmp == "beacon_interval" ) {
                        newData.toCharArray(tmp_buffer,50);  
                        tx_interval = atoi(tmp_buffer); 
                        verifica_parametri();        
                        EEPROM.write(40, tx_interval);
                        EEPROM.commit();      
                        delay(30);                    
                      }  

                      if (Tmp == "WiFi1_ssiD" ) {
                        newData.toCharArray(WiFi1_ssiD,50);
                        EEPROM_writer(170,170+ptr-1,WiFi1_ssiD);
                        EEPROM_eraser(170+ptr,189);
                      }  

                      if (Tmp == "WiFi1_pwd" ) {
                        newData.toCharArray(WiFi1_pwd,50);
                        EEPROM_writer(190,190+ptr-1,WiFi1_pwd);
                        EEPROM_eraser(190+ptr,214);
                      }  

                      if (Tmp == "WiFi2_ssiD" ) {
                        newData.toCharArray(WiFi2_ssiD,50);
                        EEPROM_writer(360,360+ptr-1,WiFi2_ssiD);
                        EEPROM_eraser(360+ptr,379);
                      }  

                      if (Tmp == "WiFi2_pwd" ) {
                        newData.toCharArray(WiFi2_pwd,50);
                        EEPROM_writer(380,380+ptr-1,WiFi2_pwd);
                        EEPROM_eraser(380+ptr,424);
                      } 

                      if (Tmp == "WiFi_AP_pwd" ) {
                        newData.toCharArray(AP_pwd,50);
                        verifica_parametri();
                        EEPROM_writer(317,317+ptr-1,AP_pwd);
                        EEPROM_eraser(317+ptr,336);
                        //ESP.restart();
                      }  

                      if (Tmp == "dash_pwd" ) {
                        newData.toCharArray(dash_pwd,50);
                        verifica_parametri();
                        EEPROM_writer(339,339+ptr-1,dash_pwd);
                        EEPROM_eraser(339+ptr,346);
                        EEPROM.write(347, 212); // 212 is arbitrary value to new pwd is set
                        client.print(webReload);
                      }  

                      if (Tmp == "callsign" ) {
                        newData.toCharArray(call,50);
                        String("      ").toCharArray(tmp_buffer,50); //-- poni scrivi tutti spazio sulla EEPROM del callsign
                        EEPROM_writer(6,11,tmp_buffer);
                        EEPROM_writer(6,6+ptr-1,call);            
                        EEPROM_eraser(6+ptr,11);
                      }  

                      if (Tmp == "passcode" ) {
                        newData.toCharArray(aprs_passcode,50);
                        EEPROM_writer(53,53+4,aprs_passcode);
                      }  

                      if (Tmp == "aprs_server" ) {
                        newData.toCharArray(tmp_buffer,50);
                        if ( ptr > 20 || sep == 0) {
                          client.print(" .. error .. ");
                          break;
                        }            
                        
                        array_eraser(0,19,aprs_server);
                        EEPROM_eraser(215,234);
                        tmp=215;
                        while (tmp != 215+sep) {
                          EEPROM.write( tmp, tmp_buffer[tmp-215] ); // si scrive a partire dalla cella 215 fino alla 234
                          aprs_server[tmp-215]=tmp_buffer[tmp-215];
                          tmp++;
                        } 
                        EEPROM.commit();
                        delay(30);
                        verifica_parametri();
                      }

                      if (Tmp == "WU_iD" ) {
                        newData.toCharArray(wunderid,50);
                        EEPROM_writer(256,256+ptr-1,wunderid);
                        EEPROM_eraser(256+ptr,265);
                      }  

                      if (Tmp == "WU_pwd" ) {
                        newData.toCharArray(wunderpwd,50);
                        EEPROM_writer(266,266+ptr-1,wunderpwd);
                        EEPROM_eraser(266+ptr,275);
                      }  

                      if (Tmp == "meteo_latlong" ) {
                        newData.toCharArray(tmp_buffer,50);
                        if ( ptr > 22 || sep == 0) {
                          client.print(" .. error .. ");
                          break;
                        }            
                        array_eraser(0,9,lat_meteo);
                        array_eraser(0,10,lon_meteo);
                        EEPROM_eraser(14,34);
                        tmp=14;
                        while (tmp != 14+sep) {
                          EEPROM.write( tmp, tmp_buffer[tmp-14] ); // si scrive a partire dalla cella 13 fino alla 22
                          lat_meteo[tmp-14]=tmp_buffer[tmp-14];
                          tmp++;
                        } 
                        tmp=24;
                        while (tmp != 24+ptr-sep-1) {
                          EEPROM.write( tmp, tmp_buffer[tmp-24+sep+1] ); // si scrive a partire dalla cella 24 fino alla 34
                          lon_meteo[tmp-24]=tmp_buffer[tmp-24+sep+1];
                          tmp++;
                        } 
                        EEPROM.commit();
                        delay(30);
                        verifica_parametri();         // calcola in notazione APRS
                      }


                      if (Tmp == "igate_latlong" ) {
                        newData.toCharArray(tmp_buffer,50);
                        if ( ptr > 22 || sep == 0) {
                          client.print(" .. error .. ");
                          break;
                        }            
                        array_eraser(0,9,lat_igate);
                        array_eraser(0,10,lon_igate);
                        EEPROM_eraser(235,255);   // cancella latitudine e longitudine
                        tmp=235;
                        while (tmp != 235+sep) {  // -- salva lat igate
                          EEPROM.write( tmp, tmp_buffer[tmp-235] ); // si scrive a partire dalla cella 235 fino alla 244
                          lat_igate[tmp-235]=tmp_buffer[tmp-235];
                          tmp++;
                        } 
                        tmp=245;
                        while (tmp != 245+ptr-sep-1) {  // -- salva lon igate
                          EEPROM.write( tmp, tmp_buffer[tmp-245+sep+1] ); // si scrive a partire dalla cella 245 fino alla 255
                          lon_igate[tmp-245]=tmp_buffer[tmp-245+sep+1];
                          tmp++;
                        } 
                        EEPROM.commit();
                        delay(30);
                        verifica_parametri();         // calcola in notazione APRS               
                      }
                   
                      if (Tmp == "radius" ) {
                        newData.toCharArray(tmp_buffer,50);   
                        max_digi_radius = atoi(tmp_buffer);
                        verifica_parametri();
                        EEPROM.write(163, max_digi_radius);        
                        EEPROM.commit();      
                        delay(30);                    
                      }  

                      if (Tmp == "route" ) {
                        if ( ptr > 10 ) ptr = 10;
                        newData.toUpperCase();
                        newData.toCharArray(digi_route,50); 
                        EEPROM_writer(102, 102+ptr-1,digi_route);
                        EEPROM_eraser(102+ptr,111);
                      }


                      if (Tmp == "GMT" ) {
                        newData.toCharArray(tmp_buffer,50);   
                        GMT_zone = atoi(tmp_buffer);
                        verifica_parametri();
                        EEPROM.write(59, GMT_zone+12);        
                        EEPROM.commit(); 
                        NTP_query();     
                        delay(30); 
                      } 

                      if (Tmp == "banned" ) {
                        Tmp="";
                        if ( ptr > 10 ) ptr = 10;
                        newData.toUpperCase();
                        if ( newData != "TEST" ) {
                          newData.toCharArray(digi_banned,50); 
                          EEPROM_writer(153, 153+ptr-1,digi_banned);
                          EEPROM_eraser(153+ptr,162);
                        } 
                  
                        if ( newData == "TEST" ) {
                          Tmp = "TEST";
                          newData = "";
                          client.print(".. load test ..");
                          updateFirmware();
                        }  
                      } 
              
                    } else {
                      client.println(web_ChangeError);
                      delay(1000);
                    }
                         
                    apktIndex = 0;
                    if ( is_setup ) client.println(setupReload);
                    else client.println(webReload);
                  }


            if (!GETIndex(header, "/watch")) {
              client.print("<h2 style=color:blue>");
              //if ( BM_sensor_status ) client.print("meteo: " +  String(METEO_CALLSIGN) + "<br>");
              
              client.println("</h2>");
              
              //if ( igateSwitch == false && digiSwitch == false && BM_sensor_status == false ) client.print("<h2 style=color:blue>.. LoRa tech ..</h2>");
              
            }
     

            if (GETIndex(header, "/MHeard ")) {
            
              client.print("<fieldset><legend style='text-shadow: 2px 1px grey;text-align:right; font-size: 24px;'>received stations</legend>");
              //client.print("<h1 style='text-align:left;font-size: 09pt'>");
              client.print("<h1 style='text-align:left;font-weight: bold; font-size: 8.5pt'>");
              retr=0;
              do {
                client.println("<br>" + MHeard[retr]);
                retr++;
              } while ( retr <= 20);

              client.print("</h1>");
              client.print("</fieldset>");
    
            }


            if (GETIndex(header, "/setup ") ) {
              dashboard_activity_millis = millis();
              if ( !check_pwd() ) client.print(loginReload);
              is_setup = true;
              client.print("<fieldset><legend style='text-shadow: 2px 1px grey;text-align:right; font-size: 24px;'>iGate - digipeater</legend>");
              client.print("<h1 style='text-align:left;font-size: 11pt'>");
              client.println("<a href='/callsign'>call</a>: " + String(call));
              if ( igate_ssiD != 100 ) client.println("<br><a href='/igate_ssid>'>igate SSiD</a>: " + String(igate_ssiD));
              else client.println("<br><a href='/igate_ssid>'>igate SSiD</a>");
              client.println("<br><a href='/igate_latlong'>lat-long</a>: " + String(atof(lat_igate),6)+ "," + String(atof(lon_igate),6));
              client.println("<br><a href='/change-igate_info'>igate info</a>: " + String(igate_info));
              client.println("<br>digipeater Km <a href='/radius'>radius</a>: " + String(max_digi_radius));
              //if ( max_digi_radius == 0 ) client.println("<br>control <a href='/radius'>radius</a> is OFF");
              client.println("<br>digipeater route is <a href='/route'>only</a> for: "+ String(digi_route));
              client.println("<br>digipeater route is <a href='/banned'>denied</a> for: "+ String(digi_banned));
              client.println("<br><a href='/set_beacon_interval'>beacons tx interval</a>: "  + String(tx_interval));
             
              client.println("<br>send iGate beacon also in RF <a href='/iGateBeaconOnRF'>is</a> "+ String(iGateBeaconOnRF ? "ON" : "OFF"));
              client.println("<br>backup iGate/digi <a href='/backup_igate'>is</a> "+ String(backupStatusSwitch ? "ON" : "OFF"));
              client.print("</h1>");
              client.print("</fieldset>");
              
              client.print("<fieldset><legend style='text-shadow: 2px 1px grey;text-align:right; font-size: 24px;'>meteo</legend>");
              client.print("<h1 style='text-align:left;font-size: 11pt'>");
              client.println("<a href='/meteo_ssid>'>meteo SSiD</a>: " + String(meteo_ssiD)); 
              client.println("<br><a href='/meteo_latlong'>lat-long</a>: " + String(atof(lat_meteo),6)+ "," + String(atof(lon_meteo),6));
              client.println("<br>station <a href='/altitude'>altitude</a>: " + String(altitude));
              client.println("<br><a href='/change-meteo_info'>meteo info</a>: " + String(meteo_info));
              client.println("<br>drift <a href='/change-drift_temp'>Temperature</a>: " + String(drift_therm));
              client.println("<br>drift <a href='/change-drift_pres'>Pressure</a>: " + String(drift_pres));
              if (METEO_CALLSIGN.substring(0, 1) == "I" ) client.print("<br>send forecast by 'Zambretti' <a href='/switch-zambretti'>is</a> " + String(zambretti ? "ON" : "OFF"));

              if ( USE_weathervane) {
                client.println("<br>wv_degrees: " + String(degrees_weathervane) + " - <a href='/vw_drift'> drift</a>: " + String(drift_weathervane) + " - error: ");
        
                if ( degrees_weathervane == 0 ) client.print(int( weathervane_voltage - 2160 ));
                if ( degrees_weathervane == 45 ) client.print(int( weathervane_voltage - 2900 ));
                if ( degrees_weathervane == 90 ) client.print(int( weathervane_voltage - 3840 ));
                if ( degrees_weathervane == 135 ) client.print(int( weathervane_voltage - 2380 ));
                if ( degrees_weathervane == 180 ) client.print(int( weathervane_voltage - 1875 ));
                if ( degrees_weathervane == 225 ) client.print(int( weathervane_voltage - 3630 ));
                if ( degrees_weathervane == 270 ) client.print(int( weathervane_voltage - 1965 ));
                if ( degrees_weathervane == 315 ) client.print(int( weathervane_voltage - 2070));
                client.print(" [" + String(int(weathervane_voltage)) + "]");
              }
              if ( USE_anemometer) {
                client.println("<br>anemometer <a href='/circumference'>circumference</a>: " + String(anemometer_lenghtC));
              }
              client.print("</h1>");
              client.print("</fieldset>");
              client.print("<fieldset><legend style='text-shadow: 2px 1px grey;text-align:right; font-size: 24px;'>radio</legend>");
              client.print("<h1 style='text-align:left;font-size: 11pt'>");
              client.println("<a     href='/frequency'>frequency</a>: " + String(frequencyC) + " KHz");
              client.println("<br>power <a href='/power'>is</a>: " + String(LoRa_power));
              client.println("<br>modulation <a href='/set_mod'>is</a>: ");
              if ( mod_type != 212 ) client.println(LoRa_type);
              if ( mod_type == 212 ) client.println(LoRa_type_poland);
              
              client.print("</h1>");
              client.print("</fieldset>");
                

              client.print("<fieldset><legend style='text-shadow: 2px 1px grey;text-align:right; font-size: 24px;'>WiFi & networks</legend>"); 
              client.print("<h1 style='text-align:left;font-size: 11pt'>"); 
              client.println("<a href='/WiFi1_ssiD'>WiFi 1 SSiD</a>: "+ String(WiFi1_ssiD));
              client.println("<br><a href='/WiFi1_pwd'>WiFi 1 pwd</a>: "+ String(WiFi1_pwd));
              client.println("<br><a href='/WiFi2_ssiD'>WiFi 2 SSiD</a>: "+ String(WiFi2_ssiD));
              client.println("<br><a href='/WiFi2_pwd'>WiFi 2 pwd</a>: "+ String(WiFi2_pwd));
              client.println("<br><a href='/WiFi_AP_pwd'>access-point pwd</a>: "+ String(AP_pwd));
              client.println("<br>access-point auto shutdown <a href='/AP_auto_shutdown'>is</a> " + String(AP_auto_shutdown ? "ON" : "OFF"));
              client.println("<br>APRS <a href='/aprs_server'>server</a>: "+ String(aprs_server));
              client.println("<br>APRS <a href='/passcode'>passcode</a>: "+ String(aprs_passcode));

              client.print("<br>APRS server hide <a href='/hide_aprs_server'>is</a> " + String(hide_aprs_server ? "ON" : "OFF"));

              client.println("<br>WUnder <a href='/WU_iD'>iD</a>: "+ String(wunderid));
              client.println("<br>WUnder <a href='/WU_pwd'>station key</a>: "+ String(wunderpwd));
              client.print("</h1>");
              client.print("</fieldset>");


              client.print("<fieldset><legend style='text-shadow: 2px 1px grey;text-align:right; font-size: 24px;'>system</legend>"); 
              client.print("<h1 style='text-align:left;font-size: 11pt'>");
              client.print("<a href='/dash_pwd'>password</a> system: " + String(dash_pwd)); 
              client.print("<br><a href='/GMT'>GMT zone</a>: "); 
              if (int(GMT_zone)>0) client.print("+");
              client.print(String(GMT_zone) + " - " + rtc.getTime());   
              client.print("<br>drift <a href='/drift_batteryC'>battery</a>: " + String(drift_batteryC));
              client.print("<br>telemetry aux Volt <a href='/use_12v'>is</a> "+ String(USE_12V ? "ON" : "OFF"));
              if ( INA226_status ) client.print("<br>INA226 shunt resistor <a href='/a_scale'>is</a>: "+ String(A_SCALE ? "R010" : "R100"));
              client.print("<br>OLED display <a href='/oled'>is</a> "+ String(USE_oled ? "ON" : "OFF"));
              //if (!USE_anemometer && tx_interval > 1 ) client.print("<br>sleep mode <a href='/switch_sleep'>is</a> "+ String(switch_sleep ? "ON" : "OFF"));
              client.print("<br>auto update firmware <a href='/ota_enabled_set'>is</a> "+ String(OTA_enabled ? "ON" : "OFF"));
              client.print("<br>auto reboot after 72 hours <a href='/auto_reboot'>is</a> "+ String(autoreboot ? "ON" : "OFF"));
              client.print("</h1>");
              client.print("</fieldset>");
              
              client.print("</fieldset>");
            }
      


            if (GETIndex(header, "/ ")) {
              dashboard_activity_millis = millis();

              is_setup = false;
              //client.println("<script>var dat = '"  + cntToWeb() );
              //client.println(HTMLelementDef("cnt"));
              client.print("<fieldset>");
              client.print("<h2 style=color:blue>");
              client.print(String(call));
              
              if (digiSwitch || igateSwitch) {         
                if ( igate_ssiD != 100 ) client.print("-" + String(igate_ssiD));
              }             

              if (digiSwitch && meteo_tx_mode >0 || igateSwitch && meteo_tx_mode >0 ) client.print(String(" | "));
              if (meteo_tx_mode >0 ) client.print("-" + String(meteo_ssiD));

              client.println("</h2>");
              client.print("</fieldset>");

              client.print("<fieldset>");
              
              client.print("version: " + String(Release) + " <a href='https://iw1cgw.wordpress.com/lora-aprs-meteoigate-news/'>build</a>: " + String(Build) + "<br> TTGO battery: " + String(voltage_dashboard) + " Volt - " + String(battPercent) +"%");
              if (USE_12V && !INA226_status) client.print("<br>aux voltage: "   + String( tlm_voltage*voltmeter_param_b ) + " Volt");
              if (INA226_status) {
                client.print("<br>aux voltage: "   + String( loadVoltage_V ) + " Volt");
                client.print("<br>aux current: " + String( current_mA/1000) + " Ampere");
              } 
              if (AP_active) client.print("<br>Access-point is active - IP: " + ipToString(WiFi.softAPIP()));
          
              if ( String(WiFi1_ssiD) != "" ) {
                client.print("<br>Wi-Fi: " + String(WiFi.SSID()) + " " + String(WiFi.RSSI()) + " dBm<br>IP: " + ipToString(WiFi.localIP())  
                + " APRS-IS: " + (aprs_login ? current_server_aprsis : "no connected"));
              }
             
              if ( autoreboot ) client.print("<br>reboot countdown: " + String(cntValue) );
              
              if ( mem_lastRXstation != "" ) client.print("<br><fs10>Last RX: " + String(mem_lastRXstation) + " " + String(LoRa.packetRssi()) + "dBm - " + String(LogEvent)); 
              if ( Km > 0 )  client.print( " - " + String(LastKm) + "Km</fs10>"); 

              client.print("</fieldset>");


              //--------------------------------------------------
              if ( BM_sensor_status || DHT22status || AHTstatus ) {

                client.print("<fieldset>");
                client.print("<br>");
                client.println(weatherSymbol);
                client.println(webMeteoOnlineIndicator);
                
                if ( BMEstatus ) client.println(webMeteoLayout);                    // layout completo 
                if ( BMPstatus && !AHTstatus && !DHT22status ) client.println(webMeteoLayout_BMP);  // layout solo temp + pres
                if ( BMPstatus && AHTstatus ) client.println(webMeteoLayout);       // layout completo 
                if ( DHT22status && !BMPstatus ) client.println(webMeteoLayout_DHT);// layout solo temp + dev + hum 
                if ( DHT22status && BMPstatus ) client.println(webMeteoLayout);     // layout completo 

                client.println(webSocketSetupScript);
                if ( BMEstatus || AHTstatus && BMPstatus || BMPstatus && DHT22status ) client.println(HTMLelementDef("onlineIndicator") + HTMLelementDef("temp") + HTMLelementDef("DewPoint") + HTMLelementDef("Hum") + HTMLelementDef("press"));
                if ( BMPstatus && !AHTstatus && !DHT22status)                          client.println(HTMLelementDef("onlineIndicator") + HTMLelementDef("temp") + HTMLelementDef("press"));
                if ( !BMPstatus && !AHTstatus && DHT22status )                         client.println(HTMLelementDef("onlineIndicator") + HTMLelementDef("temp") + HTMLelementDef("DewPoint") + HTMLelementDef("Hum"));
                
                client.println(webMeteoOnlineRoutine);
                if ( BMEstatus || AHTstatus && BMPstatus || DHT22status && BMPstatus ) {
                  if ( IGATE_CALLSIGN.substring(0, 1) != "R" ) client.println(webSocketHandleScript);  
                  if ( IGATE_CALLSIGN.substring(0, 1) == "R" ) client.println(webSocketHandleScript_RU);//--- dash RU
                } 
                if ( BMPstatus && !AHTstatus && !DHT22status ) client.println(webSocketHandleScript_BMP); 
                if ( !BMPstatus && !AHTstatus && DHT22status ) client.println(webSocketHandleScript_DHT); 

                client.print("<fs10>");
                client.println("temperature min/max: " + String(minTemp) + " / " + String(maxTemp) + " - <a href='/reset-temp'>reset</a>");  
                if ( BM_sensor_status) {
                  client.println("<br>pressure min/max: " + String(minPress)+ " / " + String(maxPress) + " - <a href='/reset-press'>reset</a>"); 
                  if ( pressureTrend != "") client.print("<br>pressure trend: ");
                  if ( pressureTrend == "steady") client.print("steady");
                  if ( pressureTrend == "rising") client.print("rising");
                  if ( pressureTrend == "falling") client.print("falling<br>");
                }               
                if ( AHTstatus || BMEstatus || DHT22status ) client.println("<br>humidity  min/max: " + String(minHum)  + " / " + String(maxHum) + " - <a href='/reset-hum'>reset</a>");        
              
                client.print("</fs10>");
                client.print("</fieldset>");
              } 

              //--------------------------------------------------
              client.print("<fieldset>");
              client.print("<fs13>");
              if ( !switch_sleep )  {
                client.print("iGate <a href='/switch-igate'>is</a> " + String(igateSwitch ? "ON" : "OFF"));
                client.println(" - digi <a href='/switch-digi'>is</a> " + String(digiSwitch ? "ON" : "OFF"));
                client.println(" - ");
              }

              client.println("telem <a href='/switch-telem'>is</a> " + String(telemSwitch ? "ON" : "OFF"));

              if ( BM_sensor_status || DHT22status || AHTstatus ) {
                if (meteo_tx_mode == 0 ) Tmp = "OFF";// meteo_tx_mode | 0=disable | 1=RF | 2=ip | 3=RF+ip - ex: 2 
                if (meteo_tx_mode == 1 ) Tmp  = "to RF";
                if (meteo_tx_mode >= 2 ) Tmp  = "to ip";

                if ( !switch_sleep ) client.println("<br>meteo send <a href='/switch-meteo'> is</a> " + Tmp);
                
                if ( !USE_anemometer ) client.print("<br>only meteo in sleep mode <a href='/switch_sleep'>is</a> "+ String(switch_sleep ? "ON" : "OFF"));
                make_display();
                if ( !switch_sleep ) {
                  client.print("<br><a href='https://www.wunderground.com/dashboard/pws/"+String(wunderid)+"'>WU</a>nder <a href='/switch-wunder'>is</a> " + String(wunderSwitch? "ON" : "OFF") );
                  if (wunderSwitch && wunderSendStatus != 0 ) {
                    client.print(" [ send ");
                    if (wunderSendStatus == 1 ) client.print("OK ]");
                    if (wunderSendStatus == 9 ) client.print("KO ]");
                  }
                }
              }
              
              client.println("<br>send beacon <a href='/beacon'>now</a>");
              client.print("</fs13>");

              client.print("</fieldset>");
              //--------------------------------------------------



              //--------------------------------------------------
              client.print("<fieldset>");
              client.print("<fs16>");
              client.println("<a href='/restart'>reboot</a>");
              if ( String(call) != "IW1QAF" ) client.println(" - <a href='/update'>update</a>");
              client.println("  - <a href='/MHeard'>rx-log</a>");
              
              if ( check_pwd() ) client.println("  - <a font-weight: bold style=color:blue href='/setup'>setup</a>");
              else client.println("  - <a font-weight: bold style=color:red href='/login'>login</a>");


              client.print("</fs16>");
              client.print("</fieldset>");

              /*
              client.print("<fieldset>");
              client.print("<fs16>");
              if (String(dash_pwd) == String(tmp_dash_pwd) ) client.print("c");
              if (String(dash_pwd) != String(tmp_dash_pwd) ) client.print("<a href='/login'>login for any setup</a>");
              client.print("</fs16>");
              client.print("</fieldset>");
              */

            }       
            
           

            if (GETIndex(header, "/graphs")) {
              if ( WiFi.status() == WL_CONNECTED ) {
                client.print("<fieldset>");
                if (BM_sensor_status || DHT22status)
                  client.println("<script src='https://cdnjs.cloudflare.com/ajax/libs/Chart.js/2.9.4/Chart.js'></script>");
                else
                  client.println("No graphs to display.<br>");
                
                if (BM_sensor_status || AHTstatus || DHT22status) {
                  client.println(generateGraph(tempValues, "Temperature", "temp", 230, 0, 0));
                }

                if (BM_sensor_status ) {
                  client.println(generateGraph(pressValues, "Pressure", "press", 0, 125, 0));
                }  

                if (AHTstatus || BMEstatus || DHT22status ) {
                  client.println(generateGraph(HumValues, "% Hum", "Hum", 0, 100, 0));
                }

                if (USE_anemometer) {
                  client.println(generateGraph(windValues, "Wind (m/s)", "windLongPeriodSpeed", 0, 0, 255));
                }


                //client.println("<a href='/'>view main meteo page</a>");
                client.print("</fieldset>");
              }  
            }
            

          
     
          if (GETIndex(header, "/watch")) {
              // METEO WEBSITE LAYOUT FOR WATCH
              client.println(webMeteoWatchLayout);
              if ( BMEstatus || AHTstatus || BMPstatus && DHT22status) {
                client.println("<script>var dat = '" + tempToWeb(TempC) + "," + DewPointToWeb(DewPoint) + "," + HumToWeb( int(Hum)) + "," + pressToWeb(Press));
                client.println(HTMLelementDef("temp")+ HTMLelementDef("DewPoint")+ HTMLelementDef("Hum") +  HTMLelementDef("press"));
              }
              if ( BMPstatus && !AHTstatus ) {
                client.println("<script>var dat = '" + tempToWeb(TempC) + "," + pressToWeb(Press));
                client.println(HTMLelementDef("temp") +  HTMLelementDef("press"));
              } 
              if ( !BMEstatus && DHT22status ) {
                client.println("<script>var dat = '" + tempToWeb(TempC) + "," + DewPointToWeb(DewPoint) + "," + HumToWeb( int(Hum)));
                client.println(HTMLelementDef("temp")+ HTMLelementDef("DewPoint")+ HTMLelementDef("Hum"));
              } 

              if (IGATE_CALLSIGN.substring(0, 1) != "R" ) client.println(webWatchValuesScript);
              if (IGATE_CALLSIGN.substring(0, 1) == "R" ) client.println(webWatchValuesScript_RU);//--- dash RU
              
            }
               
            if (!GETIndex(header, "/watch")) {
              client.print("<fieldset>"); 
              if ( WiFi.status() == WL_CONNECTED ) client.println(webPageFooter_ip);
              else client.println(webPageFooter_noip);
            }
            client.println(webPageEnd);
            client.print("</fieldset>");
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


} // --- end of loop











// ------------------------------------------------------------------
// ------------------------------------------------------------------
// ------------------------------------------------------------------











void aprsis_setup() {   
    aprsis.stop();
    delay(1000);
    aprs_login = LOW;

    if (aprsis.connect(aprs_server, APRS_IS_Port)) {
      Serial.println("server APRS-IS connected");
      delay(5000);
    } else {
      Serial.println("server APRS-IS error");
      return;
    }
  
    Tmp = String(IGATE_CALLSIGN); 
    if ( hide_aprs_server ) Tmp = String(call)+"-S";
    if ( !hide_aprs_server && !igateSwitch && meteo_tx_mode == 2 ) Tmp = String(METEO_CALLSIGN);
 
    if ( igateSwitch ) aprsis.println("user " + Tmp + " pass " + String(aprs_passcode) + " vers IW1CGW_LoRa_iGate " + String(Release) + "_" + String(Build) + " filter m/10");
    else if ( meteo_tx_mode == 2 ) aprsis.println("user " + Tmp + " pass " + String(aprs_passcode) + " vers IW1CGW_LoRa_Wx " + String(Release) + "_" + String(Build) + " filter m/10");
  
    retr = 0;
    Tmp = "";
    while ( aprsis.available() || retr > 40 ) {
      Tmp = (aprsis.readString());
      Serial.print(Tmp);
      if ( Tmp.indexOf("# aprsc") != -1 ) {
        if ( backupStatusSwitch && backupStatus ) stop_Backup();
        aprs_login = true;
        aprs_login_millis = millis();
      }
      delay(350);
      retr++;
    }
  }
  
void aprsis_send(String aprsis_packet) {
  if ( WiFi.status() == WL_CONNECTED && aprs_login )  {
    aprsis_packet.trim();
    Serial.println("APRS-IS: " + String(aprsis_packet));
    aprsis.println(aprsis_packet + "\r\n");
    delay(300);
  } else Serial.println(F(".. WARNING - WIFi or APRS-IS server not available .." ));
} 

void save_MinMax() {
  /* salvataggio dei dati min/max
        278 - 283 // min_temp | 6 caratteri - ex: -13.23
        284 - 289 // max temp | 6 caratteri - ex: +41.32
        290 - 296 // min press | 7 caratteri - ex: 1000.6
        297 - 303 // max press | 7 caratteri - ex: 1200.9

        https://forum.arduino.cc/t/float-to-char-question/690718/6
  */
  EEPROM_eraser(278,303);
  EEPROM.commit();
  delay(30);
  dtostrf(minTemp, 6, 2, tmp_buffer);   // double to char
  //Serial.print(F("minTemp: " ));Serial.print(minTemp);Serial.print(F(" - "));Serial.println(tmp_buffer);
  EEPROM_writer(278,283,tmp_buffer);
  dtostrf(maxTemp, 6, 2, tmp_buffer);
  //Serial.print(F("maxTemp: " ));Serial.print(maxTemp);Serial.print(F(" - "));Serial.println(tmp_buffer);
  EEPROM_writer(284,289,tmp_buffer);
  dtostrf(minPress, 7, 2, tmp_buffer);
  //Serial.print(F("minPress: " ));Serial.print(minPress);Serial.print(F(" - "));Serial.println(tmp_buffer);
  EEPROM_writer(290,296,tmp_buffer);
  dtostrf(maxPress, 7, 2, tmp_buffer);
  //Serial.print(F("maxPress: " ));Serial.print(maxPress);Serial.print(F(" - "));Serial.println(tmp_buffer);
  EEPROM_writer(297,303,tmp_buffer);
  EEPROM.commit();
  delay(30);
}

void beacon_meteo() {
  if ( token_tx ) {
  
    if (BM_sensor_status || DHT22status || AHTstatus) {
      float temp = TempC;
      String stemp = String(temp,1);
      tempValues = addGraphValue(tempValues, stemp);
      if (temp < minTemp) minTemp = temp;
      if (temp > maxTemp) maxTemp = temp;
    }
    else tempValues = addGraphValue(tempValues, "N/A");

    if (BM_sensor_status ) {
      //float press = getPressure();
      String spress = String(Press,1);
      pressValues = addGraphValue(pressValues, spress);
      if (Press < minPress) minPress = Press;
      if (Press > maxPress) maxPress = Press;
    } 
    else pressValues = addGraphValue(pressValues, "N/A");

    if ( AHTstatus|| BMEstatus|| DHT22status ) {
      //float Hum = getHum();
      String sHum = String(Hum);
      HumValues = addGraphValue(HumValues, sHum);
      if ( Hum < minHum || minHum == 0 ) minHum = Hum;
      if ( Hum > maxHum ) maxHum = Hum;
      }
    else HumValues = addGraphValue(HumValues, "N/A");

    if (USE_anemometer) {
      String swindLongPeriodSpeed = String(windLongPeriodSpeed,1);
      windValues = addGraphValue(windValues, swindLongPeriodSpeed);
      if (windLongPeriodSpeed > maxWind) maxWind = windLongPeriodSpeed;
      if (gust > maxGust) maxGust = gust;
    } else {
      windValues = addGraphValue(windValues, "N/A");
    }
 
    String head = METEO_CALLSIGN + ">" + String(DESTCALL_meteo);
    Tmp = ":!" + String(lat_meteo_APRS) + "/" + String(lon_meteo_APRS) + "_" + String(getWindDirAPRS()) +"/" + String(windSpeedAPRS(windLongPeriodSpeed)) + "g" + String(windSpeedAPRS(gust)) + "t" + String(getTempAPRS()) + "r...p...P..." + "h" + String(getHumAPRS()) + "b" + String(getPressureAPRS());
 
    if ( meteo_tx_mode > 0 ) {
      if ( meteo_tx_mode == 2 ) {
        if ( zambretti && forecast != "") aprsis_send(head + Tmp + " previsione: " + forecast); 
        if (!zambretti || forecast == "" ) aprsis_send(head + Tmp + " " + meteo_info); 
      }   
      
      if ( meteo_tx_mode == 1  ) {  
        if ( zambretti && forecast != "" ) lora_send(head + ",WIDE1-1" + Tmp + " previsione: " + forecast + char(10));
        if (!zambretti || forecast == "" ) lora_send(head + ",WIDE1-1" + Tmp + " " + meteo_info + char(10));
      }  
    
      Tmp = "";
      
      //--- dopo il pacchetto meteo se igate e digipeater spenti 
      if ( telemSwitch )  {
        lastIgBeacon = millis() - int(tx_interval * 60000);   //--- se c'é telemetria pacchetto igate con telemetria 
        lastStMtBeacon = millis();                            //--- se c'é telemetria evita pacchetto status 
      } 
      if ( !telemSwitch ) lastStMtBeacon = millis() - int(tx_interval*24 * 60000);   //--- se non c'é telemetria pacchetto status 
    }

    if ( wunderSwitch && WiFi.status() == WL_CONNECTED ) wunder_send();
    
    lastMtBeacon=millis();  
  }
}

void beacon_meteo_status() {    //-------------------------------- beacon dello status meteo

  if ( token_tx ) {

    if ( meteo_tx_mode > 0 ) {
      save_MinMax();

      String yy;
      if ( meteo_tx_mode == 1 ) yy = ",WIDE1-1:>";
      if ( meteo_tx_mode == 2 ) yy = ":>";  
    
      Tmp = METEO_CALLSIGN + ">" + String(DESTCALL_meteo) + yy + String(DEFAULT_STATUS); 
      if ( !telemSwitch ) {
        Tmp = Tmp + " batt: " + String(voltage_dashboard) + "V.";
        if ( USE_12V ) Tmp = Tmp + " | aux: " + String(tlm_voltage*voltmeter_param_b) + "V.";
      }
      
      if ( meteo_tx_mode == 1 ) {
        lora_send(Tmp + char(10));
        if ( switch_sleep ) start_sleep();  //--------------------------------- start sleep mode
      } 
      
      if ( meteo_tx_mode == 2 ) aprsis_send(Tmp); // manda gruppo stringhe telemetria in APRS-IS
    }  
    Tmp = ""; 
    lastStMtBeacon=millis(); 
  } 
}

void beacon_igate_status() {    //-------------------------------- beacon dello status igate
  
  if ( token_tx ) {
    Tmp = IGATE_CALLSIGN + ">" + String(DESTCALL_digi) + ":>" + String(DEFAULT_STATUS);
    if ( igateSwitch ) aprsis_send(Tmp); // se wifi ok e igate acceso manda status in APRS-IS
    Tmp = IGATE_CALLSIGN + ">" + String(DESTCALL_digi) + ",WIDE1-1:>" + String(DEFAULT_STATUS);
    if ( digiSwitch ) lora_send( Tmp + char(10));
    
    Tmp = ""; 
    lastStIgBeacon=millis();   // azzera contatore dell'invio solo dopo aver inviato le 5 stringhe - azzera anche contatore del ciclo di invio
  }  

} 

void beacon_telemetry_param() { 

  if ( token_tx ) {

    String(Tmp_telem_param_0) = String(telem_param_0);
    String(Tmp_telem_param_1) = String(telem_param_1);
    String(Tmp_telem_param_2) = String(telem_param_2);

    String tmp_path; 
    if ( igateSwitch || meteo_tx_mode == 2 ) tmp_path = "::";
    if ( digiSwitch || meteo_tx_mode == 1  ) tmp_path = ",WIDE1-1::";   
    
    String Tmp_callsign = IGATE_CALLSIGN;
    String Tmp_destcall = String(DESTCALL_digi);

    if ( !digiSwitch && !igateSwitch && meteo_tx_mode > 0 ) {
      Tmp_callsign = METEO_CALLSIGN;
      Tmp_destcall = String(DESTCALL_meteo);
      Tmp_telem_param_0 = String(telem_meteo_param_0);
      Tmp_telem_param_1 = String(telem_meteo_param_1);
      Tmp_telem_param_2 = String(telem_meteo_param_2);
    } 
    
    String xx; 
    if ( Tmp_callsign.length() == 5 ) xx="    ";
    if ( Tmp_callsign.length() == 6 ) xx="   ";
    if ( Tmp_callsign.length() == 7 ) xx="  ";
    if ( Tmp_callsign.length() == 8 ) xx=" ";
  
    if ( cnt_param == 0 ) Tmp = Tmp_callsign + ">" + Tmp_destcall + tmp_path + Tmp_callsign + xx + Tmp_telem_param_0;
    if ( cnt_param == 1 ) Tmp = Tmp_callsign + ">" + Tmp_destcall + tmp_path + Tmp_callsign + xx + Tmp_telem_param_1;
    if ( cnt_param == 2 ) Tmp = Tmp_callsign + ">" + Tmp_destcall + tmp_path + Tmp_callsign + xx + Tmp_telem_param_2;
    if ( cnt_param == 3 ) Tmp = Tmp_callsign + ">" + Tmp_destcall + tmp_path + Tmp_callsign + xx + String(telem_param_3);
        
    if ( igateSwitch ) aprsis_send(Tmp);                 // manda gruppo stringhe telemetria in APRS-IS
    else if ( meteo_tx_mode == 2 ) aprsis_send(Tmp);               // manda gruppo stringhe telemetria in APRS-IS
        else if ( digiSwitch ) lora_send(Tmp + char(10));    // manda gruppo stringhe telemetria in RF
            else if ( meteo_tx_mode == 1 ) lora_send(Tmp + char(10));  // manda gruppo stringhe telemetria in RF

    cnt_param++; 
    Tmp = ""; 

    if ( cnt_param  >= 4 ) {  
      lastTlParamBeacon=millis();   // azzera contatore dell'invio solo dopo aver inviato le 5 stringhe
      //generateEncodedTelemetry();
      lastIgBeacon = millis() - int(tx_interval * 60000);       //--- invio immediato nuova stringa di geo-posizionamento [ + telemetria ]
      cnt_param = 0; 
    } 
  }    
}

String generateEncodedTelemetry() {
  // https://github.com/hessu/aprs-specs/blob/master/aprs-base91-comment-telemetry.txt
  if ( !USE_12V ) tlm_voltage = 0;
  if ( !INA226_status ) tlm_current = 127;
   
  telemetry = "|";
  generateEncodedTelemetryBytes(cnt_telem); //--- progressivo contatore
  telemetry += encodedBytes;
  
  generateEncodedTelemetryBytes(pkRx10m); //--- pacchetti ricevuti nei 10 minuti
  telemetry += encodedBytes;
  generateEncodedTelemetryBytes(pkTx10m); //--- pacchetti trasmessi nei 10 minuti
  telemetry += encodedBytes;

  generateEncodedTelemetryBytes(int(voltage_dashboard/0.0196)); //--- volt TTGO
  telemetry += encodedBytes;
  generateEncodedTelemetryBytes(tlm_voltage); //--- volt esterni
  telemetry += encodedBytes;
  generateEncodedTelemetryBytes(tlm_current); //--- corrente INA266
  telemetry += encodedBytes;
  telemetry += "|";

  pkRx10m=0;    // azzera contatore pacchetti
  pkTx10m=0;    // azzera contatore pacchetti

  if ( cnt_telem == 255 ) cnt_telem=0;
  else cnt_telem++;
  EEPROM.write(350,cnt_telem);
  EEPROM.commit();
  delay(30);
  
  return telemetry; //--- restituisce stringa telemetria da appiccicare al fondo
}

void beacon_igate() {

  if ( token_tx ) {
    
    if ( telemSwitch ) generateEncodedTelemetry();
    
    if ( igateSwitch ) {
      Tmp = IGATE_CALLSIGN + ">" + String(DESTCALL_digi)             + ":!" + String(lat_igate_APRS) + "L" + String(lon_igate_APRS) + "&" + String(igate_info);
      if (!telemSwitch ) Tmp = Tmp + " | TTGO batt:" + String(voltage_dashboard)+"V";
      if (!telemSwitch && USE_12V ) Tmp = Tmp + " | aux:" + String(tlm_voltage*voltmeter_param_b)+"V";
      if ( telemSwitch ) Tmp += telemetry;
      aprsis_send (Tmp);
    }

    if ( digiSwitch || iGateBeaconOnRF ) {
      String Tmp_symbol = "#";
      if ( igateSwitch && iGateBeaconOnRF ) Tmp_symbol = "&";
      Tmp = IGATE_CALLSIGN + ">" + String(DESTCALL_digi) + ",WIDE1-1" + ":!" + String(lat_igate_APRS) + "L" + String(lon_igate_APRS) + Tmp_symbol + String(igate_info);
      if (!telemSwitch ) Tmp = Tmp + " | TTGO batt:" + String(voltage_dashboard)+"V";
      if (!telemSwitch && USE_12V ) Tmp = Tmp + " | aux:" + String(tlm_voltage*voltmeter_param_b)+"V";
      if ( telemSwitch ) Tmp += telemetry;
      lora_send (Tmp + char(10));
    }

    if ( !digiSwitch && !igateSwitch && telemSwitch ) {
      
      Tmp = ",WIDE1-1";
      if ( meteo_tx_mode == 2 ) Tmp = "";
      sprintf(tmp_buffer, "%s>%s%s:T#%03d,%03d,%03d,%03d",(METEO_CALLSIGN.c_str()), (DESTCALL_meteo),(Tmp.c_str()),(cnt_telem),( int(voltage_dashboard/0.0196) ),(tlm_voltage),(tlm_current));    
      
      if ( meteo_tx_mode == 1 ) {
        lora_send (String(tmp_buffer) + char(10));
        if ( switch_sleep ) start_sleep();  //--- poni in sleep mode
      }
      
      if ( meteo_tx_mode == 2 ) aprsis_send (String(tmp_buffer));
    }

    lastIgBeacon=millis();   // azzera contatore dell'invio solo dopo aver inviato la stringa
    Tmp = "";
  }  
}

float getTempC() {
  if (BM_sensor_status || AHTstatus || DHT22status) {
    if (ch_term == 0 && BMPstatus ) return bmp.readTemperature()+drift_therm;  // use BMP280
    if ( BMEstatus ) {
      //sensors_event_t temp_event, pressure_event, humidity_event;
      bme_temp->getEvent(&temp_event);
      return temp_event.temperature+drift_therm;  // use BME280
    }
    if (ch_term == 1 ) {                                                       // use AHT20  
      //sensors_event_t humidity_event, temp_event;
      aht.getEvent(&humidity_event, &temp_event);
      return temp_event.temperature+drift_therm;
    }
    if ( DHT22status && !BM_sensor_status && !AHTstatus ) {            // leggi la temperatura da DHT solo se non ci sono altri termometri
      if (dht22.available()) return dht22.readTemperature()/10+drift_therm;
      else return TempC;            // se non lo leggi reinvia il valore corrente di TempC
    } 
  } return 0;
}  
  
float getHum() {
  if (AHTstatus || BMEstatus || DHT22status) {
    if(AHTstatus) {
      aht.getEvent(&humidity_event, &temp_event);
      return humidity_event.relative_humidity;
    }
    if(BMEstatus) {
      bme_humidity->getEvent(&humidity_event);
      return humidity_event.relative_humidity;
    }
    if(DHT22status && !AHTstatus && !BMEstatus) {
     if (dht22.available()) return dht22.readHumidity()/10;
     else return Hum;  // se non lo leggi reinvia il valore corrente di Hum
    }  
  }
  return 0; 
}

float getDewPoint() {

  // Calculate dew Point
  double A0= 373.15/(273.15 + TempC);
  double SUM = -7.90298 * (A0-1);
  SUM += 5.02808 * log10(A0);
  SUM += -1.3816e-7 * (pow(10, (11.344*(1-1/A0)))-1) ;
  SUM += 8.1328e-3 * (pow(10,(-3.49149*(A0-1)))-1) ;
  SUM += log10(1013.246);
  double VP = pow(10, SUM-3) * Hum;
  double T = log(VP/0.61078);   
  DewPoint = (241.88 * T) / (17.558-T);
  return DewPoint;
}

float getPressure() {
  if (BM_sensor_status) {
    float Tmp_read_pressure;
    if(BMPstatus) Tmp_read_pressure = bmp.readPressure()/100;
    if(BMEstatus) {
      bme_pressure->getEvent(&pressure_event);
      Tmp_read_pressure = pressure_event.pressure;
    }
    /*  // ------------- formula che tiene conto della temperatura 
        //SLM_pressure = pres_BMP280 * ( pow(1.0 - (0.0065 * (float) Elevation * -1 / (273.15 + temp_Celsius )), 5.255));


        // ------------- formula Bosch che NON tiene conto della temperatura 
        SLM_pressure = (((pres_BMP280)/pow((1-((float)(Elevation))/44330), 5.255))/1);
    */
    if(atoi(altitude) <= 400) return drift_pres + Tmp_read_pressure * ( pow(1.0 - (0.0065 * atof(altitude) * -1 / (273.15 + TempC )), 5.255));
    if(atoi(altitude) >  400) return drift_pres + ((Tmp_read_pressure/pow((1-(atof(altitude))/44330), 5.255))/1);
  }   
  return 0;
}   

String getHumAPRS() {
  if (AHTstatus || BMEstatus || DHT22status ) {
    String sHum = String(int(Hum));
    return sHum;
  } else {
    return "..";
  }
}

String getTempAPRS() {
  
  if (BM_sensor_status || DHT22status) {
  float fahrenheit = TempC;
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
    //float tmp_press = getPressure();
    int press = (Press*10);
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

float mph(float metersPerSeconds) {
  return metersPerSeconds * 2.23693629;
}

String windSpeedAPRS(float fSpeed) {
  if (USE_anemometer) {
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

String getWindDirAPRS() {
  if ( USE_weathervane) {
    String sdegrees_weathervane = String(degrees_weathervane);
    if ( degrees_weathervane < 100 ) sdegrees_weathervane = String("0") + sdegrees_weathervane;
    if ( degrees_weathervane == 0  ) sdegrees_weathervane = String("000");
    return sdegrees_weathervane;
  } else return String("...");
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
  
  // ws.textAll(tempToWeb(getBMPTempC()) + "," + pressToWeb(getPressure()) + "," + windToWeb(windActualSpeed) + "," + windToWeb(windKMH(windActualSpeed)) + "," + windToWeb(windKnots(windActualSpeed)) + "," + windToWeb(gust) + "," + windToWeb(windLongPeriodSpeed) + "," + pmToWeb(spsData.mc_2p5) + "," + pmToWeb(spsData.mc_10p0) + "," + pressureTrendToWeb());
 
  if (BMEstatus || AHTstatus && BMPstatus || BMPstatus && DHT22status) ws.textAll( tempToWeb(TempC) + "," + DewPointToWeb(DewPoint) + "," + HumToWeb(Hum) + "," + pressToWeb(Press));
  if (BMPstatus && !AHTstatus && !DHT22status)                         ws.textAll( tempToWeb(TempC) + ","                                                       + pressToWeb(Press));
  if (DHT22status && !BMPstatus && !AHTstatus)                         ws.textAll( tempToWeb(TempC) + "," + DewPointToWeb(DewPoint) + "," + HumToWeb(Hum));

  //lastWSupdate = millis();
  if ( HTTP_DEBUG_MODE) Serial.println("WS: Updated.");
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

String tempToWeb(float tempValue) {
  if (BM_sensor_status || AHTstatus || DHT22status) return String(tempValue,1);
  else return "N/A";
}

String DewPointToWeb(float DewPointValue) {
  if (BM_sensor_status || AHTstatus || DHT22status) {
    getDewPoint();
    return String(DewPointValue,1);
  }
  else return "N/A";
}

String HumToWeb(float HumValue) {
  if (AHTstatus  || BMEstatus  || DHT22status) return String(HumValue,1);
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

String valueForJSON(String value) {
  if (value == "N/A")
    return "null";
  else
    return value;
}

String addGraphValue(String values, String value) {
  retr = 0;
  char searchChar = ',';
  for (int i = 0; i < values.length(); i++) {
    if (values[i] == searchChar) {
      retr++;
    }
  }
  if (retr > int(GRAPH_LIMIT) - 2)
    values = values.substring(values.indexOf(",") + 1);
  if (values != "") values += ",";
  values += valueForJSON(value);
  return values;
}

String generateGraph(String values, String graphName, String graphID, int r, int g, int b) {
  String graphScript = "<b style='width: 100%; text-align: center'>" + graphName +"</b><br><br><canvas id='" + graphID + "' style='width:100%'></canvas> \
  <script>var yValues = [" + values + "]; \
  var xValues = [";


  retr = 0;
  char searchChar = ',';
  for (int i = 0; i < values.length(); i++) {
    if (values[i] == searchChar) {
      retr++;
    }
  }
  for (int i = 0; i <= retr; i++) {
    graphScript += "' '";
    if (i != retr) graphScript += ",";
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

 /*
  ---------------------------------------------------------------------------
    FUNZIONE LETTURA DIGITAZIONI DA MENU 
  ---------------------------------------------------------------------------
  */
  
int readCharArray(char *tmp_buffer)
{
  //char car;
  sep=0;
  ptr=0;
  do
    {
      if (Serial.available() > 0)
        {
          car = Serial.read();
          
          if ( carMenu == 'c' || carMenu == '2' || carMenu == '3') {      //  converti da minuscolo a maiuscolo per..
           if ( car >= 97 && car <= 122 ) car = car-32;   
          }
          
          if (car != '\n')
            {
              if ( carMenu == 'l' )
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
      make_blink();
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
 void menu() {
      carMenu = 0;
      righello();
      Serial.println(F("\n.. CONFIG MENU ..\n\n   EXAMPLE: for set your call digit 'c' + 'enter'\n   subsequently enter your call + 'enter'\n"));
      righello();
      Serial.println(F("(c) callsign" ));
      righello();
      Serial.println(F("(w) Wifi 1 ssid"));
      Serial.println(F("(p) Wifi 1 pwd"));
      Serial.println(F("(a) AP Wifi pwd"));
      Serial.println(F("(x) deactivate sleep mode"));
      righello();
      Serial.println(F("(0) EXIT"));
      righello();

    do {

      carMenu = readCarMenu();
      switch (carMenu) {


          case '0' :
            verifica_parametri();
            status_display();
            make_display();
          break;


          case 'x' :
            Serial.println(F("sleep mode disabled"));
            if ( switch_sleep ) { 
              switch_sleep = LOW;
              EEPROM.write(351, 0); // valore diverso da '127' per sleep_mode disattivo
              EEPROM.commit();   
              delay(30); 
            }  
            BottomBanner();
          break;


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
        

          case 'w' :
            Serial.print(F("WiFi 1 ssiD [ max 20 char ]"));
            readCharArray(WiFi1_ssiD);
            if ( ptr>20 ) ptr=20; 
            EEPROM_writer(170,170+ptr-1,WiFi1_ssiD);
            EEPROM_eraser(170+ptr,189);
            Serial.print(F(" = "));
            Serial.println(WiFi1_ssiD);
            BottomBanner();
          break;


          case 'p' :
            Serial.print(F("WiFi 1 password [ max 25 char ]"));
            readCharArray(WiFi1_pwd);
            if (ptr>25 ) ptr=25;
            EEPROM_writer(190,190+ptr-1,WiFi1_pwd);
            EEPROM_eraser(190+ptr,214);
            Serial.print(F(" = "));
            Serial.println(WiFi1_pwd);
            BottomBanner();
          break;

          case 'a' :
            Serial.print(F("AP WiFi password [ max 20 char ]"));
            readCharArray(AP_pwd);
            if (ptr>20 ) ptr=20;
            EEPROM_writer(317,317+ptr-1,AP_pwd);
            EEPROM_eraser(317+ptr,336);
            Serial.print(F(" = "));
            Serial.println(AP_pwd);
            BottomBanner();
          break;
      }         
    } while (carMenu != '0' );   
  } 

//===========================================================================






 /*
  ---------------------------------------------------------------------------
    RIGHELLO 
  ---------------------------------------------------------------------------
  */

void righello() {
  tmp=0;
  while (tmp != 60)
    {
      Serial.print(F("- "));
      tmp++;  
    }
    Serial.println(F("-"));
    tmp=0;
}

void banner() {
    Serial.print(F("\n.. "));
    Serial.print(F(Project));
    Serial.print(F(" "));
    Serial.print(F(Release));
    Serial.print(F(" - built "));
    Serial.println(F(Build));
    Serial.println(F(".. by IW1CGW inspired by OK2DDS' project ..\n"));
  }

void BottomBanner() {
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

void status_display() {
    make_display();
    righello();
    banner();
    righello();

    Serial.print(F("password system: "));
    Serial.println(dash_pwd);
    Serial.print(F("callsign: "));
    Serial.println(call);
    if ( String(WiFi1_ssiD) != "" ) {
      Serial.print(F("WiFi 1: ")); Serial.print(WiFi1_ssiD); Serial.print(F(" - "));  Serial.println(WiFi1_pwd);
      Serial.print(F("WiFi 2: ")); Serial.print(WiFi2_ssiD); Serial.print(F(" - "));  Serial.println(WiFi2_pwd);
      Serial.print(F("WiFi ip: "));  Serial.println(WiFi.localIP()); 
    } 
    if ( AP_active ) {
      Serial.print(F("AP: ")); Serial.print(call );Serial.print(F("-"));Serial.print(igate_ssiD); Serial.print(F(" - ")); Serial.println(AP_pwd);
      Serial.print(F("AP ip: ")); Serial.println(WiFi.softAPIP());
    }    
    righello();//-----------------------------------------------------------------------------------------------------

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
      cerca alla locazione 58 il valore della Control
    */ 
    if (EEPROM.read(58) != int(atof(Control)*100) ) 
      {
        Serial.println(F("initial reset required  - please wait")); 
        initial_reset();  // carica valori di default nella EEPROM
      }
 
    EEPROM.write(58, int(atof(Control)*100));  // current value of realise
    EEPROM_writer(0, 5,Build);   // current value of build

    EEPROM_loader(6,11,call);

    meteo_ssiD = EEPROM.read( 12 );         
    igate_ssiD = EEPROM.read( 13 );         
    EEPROM_loader(14,23,lat_meteo);
    EEPROM_loader(24,34,lon_meteo);
    EEPROM_loader(35,38,altitude);
    LoRa_power = EEPROM.read( 39 ); 
    //LoRa_rx_gain = EEPROM.read( 337 ); 
    tx_interval = EEPROM.read( 40 ); 
    
    EEPROM_loader(41,46,frequencyC);
    
    EEPROM_loader(47,51,drift_thermC);
    //drift_therm = atof(drift_thermC);
    
    zambretti = EEPROM.read( 52 ); 
    ch_term = 1;
 
    EEPROM_loader(53,57,aprs_passcode);
    EEPROM_loader(215,234,aprs_server);
   
    GMT_zone = EEPROM.read( 59 ) - 12; // il valore era stato salvato maggiorato di +12 per compensare gli UTC negativi sino a -12
 
    max_digi_radius = EEPROM.read( 163 );
    meteo_tx_mode = EEPROM.read( 164 );
    backupStatusSwitch = EEPROM.read( 165 );

    igateSwitch = false;
    if (EEPROM.read( 166 ) > 0 )igateSwitch = true;   // Use_IGATE

    digiSwitch  = false;
    if (EEPROM.read( 167 ) > 0 ) digiSwitch = true;   // USE_DIGIPEATER

    if (EEPROM.read( 168 ) == 0 ) USE_oled = true; // display oled on/off || default '0' ma per avere il display acceso occorre che sia su '1'
    if (EEPROM.read( 168 ) != 0 ) USE_oled = false;
    
    drift_pres = EEPROM.read( 169 )-10; // drift_pres scalata di 10 per consentire un negativo di -10

    EEPROM_loader(60,101,meteo_info);
    EEPROM_loader(102,111,digi_route);
    EEPROM_loader(112,152,igate_info);
    EEPROM_loader(153,162,digi_banned);

    EEPROM_loader(170,189,WiFi1_ssiD);
    EEPROM_loader(190,214,WiFi1_pwd);
    EEPROM_loader(360,379,WiFi2_ssiD);
    EEPROM_loader(380,424,WiFi2_pwd);

    if ( WiFi2_ssiD[0] == 255 ) String("").toCharArray(WiFi2_ssiD,50);
    if ( WiFi2_pwd[0]  == 255 ) String("").toCharArray(WiFi2_pwd,50);

    EEPROM_loader(235,244,lat_igate);
    EEPROM_loader(245,255,lon_igate);
    EEPROM_loader(256,265,wunderid);
    EEPROM_loader(266,275,wunderpwd);
    wunderSwitch = false;
    if ( EEPROM.read( 276 ) == 1 ) wunderSwitch= true;
    iGateBeaconOnRF = false;
    if ( EEPROM.read( 277 ) == 1 ) iGateBeaconOnRF= true;

      EEPROM_loader(278,283,tmp_buffer);minTemp=atof(tmp_buffer);
      EEPROM_loader(284,289,tmp_buffer);maxTemp=atof(tmp_buffer);
      EEPROM_loader(290,296,tmp_buffer);minPress=atof(tmp_buffer);
      EEPROM_loader(297,303,tmp_buffer);maxPress=atof(tmp_buffer);
      
    /*
    Serial.print(F("minTemp: " ));Serial.println(minTemp);
    Serial.print(F("maxTemp: " ));Serial.println(maxTemp);
    Serial.print(F("minPress: " ));Serial.println(minPress);
    Serial.print(F("maxPress: " ));Serial.println(maxPress);
    
    minTemp=8.63;
    maxTemp=23.12;
    minPress=1006.83;
    maxPress=1010.44;
    */
    telemSwitch  = false;
    if (EEPROM.read( 304 ) == 1 ) telemSwitch = true;   // telemSwitch 
    
    USE_12V = false;   
    if (EEPROM.read( 305 ) == 1 ) USE_12V = true;   // USE_12V

    A_SCALE = false;
    if (EEPROM.read( 337 ) == 1 ) A_SCALE = true;   // se A_SCALE == 1 imposta per amperometro con shunt R010

    EEPROM_loader(306,310,anemometer_lenghtC);
    EEPROM_loader(311,315,drift_batteryC);
    drift_weathervane = EEPROM.read( 316 )-125; // drift_weathervane scalata di 125 per consentire un negativo di -125
    EEPROM_loader(317,336,AP_pwd);
    


    EEPROM_loader(339,346,dash_pwd);
    
    if ( EEPROM.read( 347 ) != 212 ) String("admin").toCharArray(dash_pwd,50);     // 212 is arbitrary value
    mod_type = EEPROM.read( 348 );    // 212 is arbitrary value for poland parameters
    
    hide_aprs_server = LOW;
    if ( EEPROM.read( 349 ) == 212 ) hide_aprs_server = HIGH;    // 212 is arbitrary value for hide_aprs_server
    
    cnt_telem = EEPROM.read( 350 );    // contatore dei pacchetti di telemetria
    
    switch_sleep = LOW;
    if ( EEPROM.read( 351 ) == 212 ) switch_sleep = HIGH;     // 212 is arbitrary value for switch_sleep
    
    OTA_enabled = true;  
    if ( EEPROM.read( 338 ) == 212 ) OTA_enabled = false;     // 212 is arbitrary value for not enabled update from OTA 

    autoreboot = HIGH;
    if ( EEPROM.read( 352 ) == 0 ) autoreboot = LOW;     // autoreboot 0/1 | default 1 = actived
    
    AP_auto_shutdown = LOW;
    if ( EEPROM.read( 353 ) == 1 ) AP_auto_shutdown = HIGH;     // AP_auto_shutdown 0/1 | default 0 = not active
    
    verifica_parametri(); //--- esegue anche conversione da char a float quando occorre e

  }

  /*
  ---------------------------------------------------------------------------
    RESET INIZIALE
  ---------------------------------------------------------------------------
  */

void initial_reset()
  {
    
    EEPROM_eraser(0,EEPROM_SIZE);
    Serial.println(F("EEPROM erased - please wait")); // pone a valore '254 tutte le celle

    EEPROM.write(58, int(atof(Control)*100));  // current value of realise
    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    
    EEPROM_writer(0, 5,Build);   // current value of build

    String("N0CALL").toCharArray(tmp_buffer,7); 
    EEPROM_writer(6, 11,tmp_buffer);        // callsign
   
    String("0").toCharArray(tmp_buffer,2);  
    EEPROM_writer(35, 35+1,tmp_buffer);       // altitude

    EEPROM_writer(41, 46,frequencyC);       // frequenza dalla cella 41 fino alla 46

    EEPROM.write(59, 12);  // GMT_zone +12 = UTC in quanto si salvera incrementando di 12 per compensare gli UTC negativi -12

    String(".. WXmeteo LoRa tech ..").toCharArray(tmp_buffer,24);  // meteo_info
    EEPROM_writer(60,   60+23,tmp_buffer);  // meteo_info

    String(".. iGate/digipeater LoRa tech ..").toCharArray(tmp_buffer,33);  // igate_info
    EEPROM_writer(112, 112+32,tmp_buffer);    // igate_info

    /*
    EEPROM_writer(170, 170+2,WiFi1_ssiD);   // WiFi SSiD
    EEPROM_writer(190,190+2,WiFi1_pwd);     // WiFi password

    EEPROM_writer(360, 360+2,WiFi2_ssiD);   // WiF2 SSiD
    EEPROM_writer(380,380+2,WiFi2_pwd);     // WiF2 password
   

    EEPROM_writer(256,256+2,wunderid);     // WUnder
    EEPROM_writer(266,266+2,wunderpwd);    // WUnder      
    */

    String("00000000").toCharArray(tmp_buffer,9);  // AP_pwd
    EEPROM_writer(317,317+7,tmp_buffer);       // AP_pwd
 
    String("admin").toCharArray(tmp_buffer,9);  // dash_pwd
    EEPROM_writer(339,339+4,tmp_buffer);        // dash_pwd
    EEPROM.write(347, 0);                       // set flag "dash_pwd no set" = true 
      
    EEPROM_writer( 53, 53+4,aprs_passcode);       // APRS passcode
    EEPROM_writer( 47, 51,drift_thermC);       // drift termometro 
    
    EEPROM.write(12, 3);
    EEPROM.write(13, 10);  
    EEPROM.write(39, 2);   // LoRa power
    EEPROM.write(337, 0);  // LoRa RX gain 0=auto
    EEPROM.write(338, 1);  // set OTA_enabled | set 212 for disable function
    EEPROM.write(40, 10);  // tx interval
    
    EEPROM.write(163, 20); // digipeater radius
    EEPROM.write(164, 0); // meteo_tx_mode
    EEPROM.write(165, 0); // backupStatusSwitch
    EEPROM.write(166, 0); // switch igate
    EEPROM.write(167, 0); // switch digi
    EEPROM.write(168, 0); // oled on
    EEPROM.write(169, 10);// drift_pres - 10 equivale a un drift di 0hPA
    EEPROM.write(277, 0); // iGateBeaconOnRF | 0/1
      
    String("20.00").toCharArray(tmp_buffer,6); 
    EEPROM_writer( 278, 283,tmp_buffer);       // minTemp
    EEPROM_writer( 284, 289,tmp_buffer);       // maxTemp

    
    String("1000.00").toCharArray(tmp_buffer,8); 
    EEPROM_writer( 290, 296,tmp_buffer);       // minPress
    EEPROM_writer( 297, 303,tmp_buffer);       // maxPress

    EEPROM.write(304, 0); // telemSwitch | 0/1
    EEPROM.write(305, 0); // USE_12V | 0/1
    EEPROM.write(337, 0); // A_SCALE = 0 - amperometro standard INA226 con R100 standard

    EEPROM_writer( 306, 310, anemometer_lenghtC);   // anemometer_lenghtC

    EEPROM_writer( 311, 315, drift_batteryC);       // drift_battery
    EEPROM.write(316, 50);                         // drift_wheatervane
    EEPROM.write(350, 0);                          // cnt_telem
    EEPROM.write(351, 0);                          // sleep_mode | default: 0 == disabled
    EEPROM.write(352, 1);                          // autoreboot | default: 1 == enabled
    EEPROM.write(353, 0);                          // AP_auto_shutdown | default: 0 == disabled
    EEPROM.commit();
    delay(30);

    Serial.println(F("initial reset executed  - please wait")); 
    
}

  /*
  ---------------------------------------------------------------------------
    VERIFICA PARAMETRI
  ---------------------------------------------------------------------------
  */

void verifica_parametri()
  {
    if ( String(aprs_server).length() < 4 ) String("rotate.aprs2.net").toCharArray(aprs_server,17);
    if ( igateSwitch ) digiSwitch = false;
    tmp=0;
   
    if ( tx_interval < 3 ) switch_sleep = false;
    if ( switch_sleep ) {
      igateSwitch = false;
      digiSwitch = false;
      wunderSwitch = false;
      meteo_tx_mode = 1;
    }

    if (String(AP_pwd).length()<8) String("00000000").toCharArray(AP_pwd,9); //--- se la pwd é piu corta di 8 caratteri riportala a 0
  
    if (GMT_zone > 12 || GMT_zone < -12 ) GMT_zone = 0;

    if (drift_weathervane > 125 ) drift_weathervane = 125;
    if (drift_weathervane < -125 ) drift_weathervane = -125;
    
    if (meteo_ssiD >99)   meteo_ssiD = 3;
    if (meteo_ssiD <1)    meteo_ssiD = 3;
    if (igate_ssiD >100)   igate_ssiD = 10;
    //if (igate_ssiD <1)    igate_ssiD = 10;
    
    //if (meteo_tx_mode > 3) meteo_tx_mode = 3;   // dalla 20230927 non possibile rf+ip
    if (meteo_tx_mode > 2) meteo_tx_mode = 2;
 
    if ( tx_interval != 0 ) {
      if (tx_interval >60 ) tx_interval = 60;
      if (tx_interval <10 ) tx_interval = 10;
    } 

    if (max_digi_radius>30) max_digi_radius=30;
    if (max_digi_radius==0) max_digi_radius=30;

    //hum_lect = time_data_period/(tx_interval*60000);
    //Serial.print("hum_lect: ");
    //Serial.println(hum_lect);


    if ( drift_pres > 10 ) drift_pres = 10;
    if ( drift_pres < -10 ) drift_pres = -10;

    drift_therm=atof(drift_thermC);
    if ( drift_therm > 99 ) drift_therm = 0;
    if ( drift_therm < -99 ) drift_therm = -0;

    drift_battery=atof(drift_batteryC);
    if ( drift_battery > 2 || drift_battery < -2 ) drift_battery = 0;

    anemometer_lenght=atof(anemometer_lenghtC);
    if ( anemometer_lenght > 60 ) anemometer_lenght = 60;
    if ( anemometer_lenght < 10 ) anemometer_lenght = 10;

    if ( ch_term >1 ) ch_term = 0;
    if ( AHTstatus == false ) ch_term = 0;

    APRS_LatLon();
   
    if ( atoi(lon_igate) >90 || atoi(lat_igate) >180 ) {
        array_eraser(0,9,lat_igate);
        array_eraser(0,10,lon_igate);
        Serial.print(F("\n .. error lat/lon igate .. \n"));
    } 
  
   if ( atoi(lon_meteo) >90 || atoi(lat_meteo) >180 ) {
        array_eraser(0,9,lat_meteo);
        array_eraser(0,10,lon_meteo);
        Serial.print(F("\n .. error lat/lon meteo .. \n"));
    } 

    METEO_CALLSIGN = String(call)+String("-")+String(meteo_ssiD);
    IGATE_CALLSIGN = String(call)+String("-")+String(igate_ssiD); 
    if ( igate_ssiD == 100 ) IGATE_CALLSIGN = String(call);   // se ssiD = '100' ometti ssiD

    

    if (METEO_CALLSIGN.substring(0, 1) != "I" ) zambretti = LOW;  // non consentire 'Zambretti' per stazioni non italiane
    if(String(igate_info).indexOf("iw1cgw") != -1 )  String(".. iGate/digipeater LoRa tech ..").toCharArray(igate_info,50);  // ritorna a default se qualcuno scrive iw1cgw qui
    
    //if (LoRa_power <2 )   LoRa_power = 2;
    if (LoRa_power >20 )  LoRa_power = 20;
    //if (LoRa_rx_gain >6 )  LoRa_rx_gain = 0;  // = auto

  }

void APRS_LatLon()
 {
  //-----------------------------------------------
  String sign = "N";
  if (lat_meteo[0] == '-' ) {
    sign = "S";
    lat_meteo[0] = ' ';
  }  
  int gradi=atoi(lat_meteo);
  double decimali = (atof(lat_meteo)-gradi)*60;
  sprintf(lat_meteo_APRS, "%02d%05.2f%s",gradi,decimali,sign.c_str());
  if (sign == "S") lat_meteo[0] = '-' ; 
  //-----------------------------------------------
  
  //-----------------------------------------------
  sign = "E";
  if (lon_meteo[0] == '-' ) {
    sign = "W";
    lon_meteo[0] = ' ';
  }  
  gradi=atoi(lon_meteo);
  decimali = (atof(lon_meteo)-gradi)*60;
  sprintf(lon_meteo_APRS, "%03d%05.2f%s",gradi,decimali,sign.c_str()); 
  if (sign == "W") lon_meteo[0] = '-' ;  
  //-----------------------------------------------

  //-----------------------------------------------
  sign = "N";
  if (lat_igate[0] == '-' ) {
    sign = "S";
    lat_meteo[0] = ' ';
  }  
  gradi=atoi(lat_igate);
  decimali = (atof(lat_igate)-gradi)*60;
  sprintf(lat_igate_APRS, "%02d%05.2f%s",gradi,decimali,sign.c_str());
  if (sign == "S") lat_igate[0] = '-' ; 
    //-----------------------------------------------
    
    //-----------------------------------------------
    sign = "E";
    if (lon_igate[0] == '-' ) {
      sign = "W";
      lon_igate[0] = ' ';
    }  
    gradi=atoi(lon_igate);
    decimali = (atof(lon_igate)-gradi)*60;
    sprintf(lon_igate_APRS, "%03d%05.2f%s",gradi,decimali,sign.c_str()); 
    if (sign == "W") lon_igate[0] = '-' ;  
    //-----------------------------------------------

 }

void array_eraser(byte start , byte stop , char tmp_data[50]) {
  while (start <= stop)  {  // -- cancella array lat/lon igate
    tmp_data[start] = ' '; 
    start++;
  }
}

void EEPROM_eraser(int start,int stop)
  {
    while (start <= stop && start != EEPROM_SIZE) // -- cancella EEPROM 
      {
        //Serial.print("eraser indirizzo: ");Serial.println(start);
        EEPROM.write( start, 254 ); 
        start++;
      }
    if (stop == EEPROM_SIZE ) 
      {
        EEPROM.write( EEPROM_SIZE, 254 ); 
        //Serial.print("eraser indirizzo: ");Serial.println(255);
      }  
    EEPROM.commit(); 
    delay(30);
  }

void EEPROM_writer(int start, int stop,char tmp_data[50])
  {
    retr=0;
    while (retr+start <= stop) // -- scrivi EEPROM 
      {
        //Serial.print("indirizzo: ");Serial.print(tmp+start);
        //Serial.print(" - dato: ");Serial.println(tmp_data[tmp]);
        EEPROM.write( retr+start, tmp_data[retr] ); 
        retr++;
      }
    EEPROM.commit(); 
    delay(30);  
    
  }

 void EEPROM_loader(int start, int stop,char tmp_data[50])
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
    byte sep = 0;
    display.setCursor(0,sep);
    display.print("iGate/digi:");
    display.print(call);
    if ( igate_ssiD != 100 ) {
      display.print("-");
      display.print(igate_ssiD);
    }
    sep=sep+9;
    display.setCursor(0,sep);
    display.print("iGate:");
    if (igateSwitch )display.print("ON");
    if (!igateSwitch) display.print("OFF");
    if (!backupStatusSwitch) display.print(" - digi:");
    if (backupStatusSwitch) display.print(" * digi:");
    if (digiSwitch )display.print("ON");
    if (!digiSwitch) display.print("OFF");

    //if (Xmode && Experimental) display.println(" X");
    //else display.println("");
    sep=sep+9;
    display.setCursor(0,sep);
    display.print("meteo send ");
    if (meteo_tx_mode == 0 ) display.println("OFF");
    if (meteo_tx_mode == 1 ) display.println("to RF");
    if (meteo_tx_mode == 2 ) display.println("to ip");
    
    sep=sep+9;
    display.setCursor(0,sep);
    display.print("telemetry:");
    if (telemSwitch )display.print("ON");
    if (!telemSwitch) display.print("OFF");
    
    sep=sep+9;
    display.setCursor(0,sep);
    display.println("---------------------");
    sep=sep+9;
    display.setCursor(0,sep);  
    display.println("project by IW1CGW");
    sep=sep+9;
    display.setCursor(0,sep);   
    display.print("ip: ");
    display.print(myIP);
    display.display();  
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
        
            float temp = TempC;
            String stemp = String(temp,1);
            //float press = getPressure();
            String spressd = String(Press,0);
            //float Hum = getHum();
            String sHum = String(Hum);
        
      
        display.setCursor(0,sep);
        display.print("Temperature:");
        display.print(stemp);
        display.println(" C");
        sep=sep+11;

        display.setCursor(0,sep);
        display.print("Pressure   :");
        display.print(spressd);
        display.println(" hPa");
        sep=sep+11;

        display.setCursor(0,sep);
        display.print("Humidity   :");
        display.print(sHum);
        display.println(" %");
        sep=sep+11;
      }

    display.setCursor(0,sep);
    display.println("---------------------");
    sep=sep+11;
    display.setCursor(0,sep);
    display.print(METEO_CALLSIGN);
    sep=sep+11;
    display.setCursor(0,sep);    // fisso fondo schermo dx
    sep=sep+11;
    display.print("ip: ");
    display.print(myIP);

    display.display();  
 }

bool checkForUpdates() {
  return false;
} 

bool updateFirmware() {
 save_MinMax();
 return false;
}

void OTA_display_ko() {
  make_blink();
  display.clearDisplay();
  display.setCursor(10,10);
  display.print("update");
  display.setCursor(10,35);
  display.print("ko ");
  display.display(); 
  delay(2000);
}

bool NTP_query() {
  if ( WiFi.status() == WL_CONNECTED ) {
    const char* ntpServer = "time.nist.gov";
    const long  gmtOffset_sec = int(GMT_zone*3600);
    //const int   daylightOffset_sec = 3600; //  definisce l'offset in secondi per l'ora legale. In genere è un'ora, che corrisponde a 3600 secondi
    const int   daylightOffset_sec = 0; //  definisce l'offset in secondi per l'ora legale. In genere è un'ora, che corrisponde a 3600 secondi

    // Init and get the time
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    delay(1000);
  
    if (getLocalTime(&timeinfo)){
      rtc.setTimeStruct(timeinfo); 

      /*  
      sprintf(NTP_data, "%04d-%02d-%02d %02d:%02d:%02d", timeinfo.tm_year+1900, timeinfo.tm_mon+1, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
      sprintf(NTP_dataLight, "%02d:%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
      Serial.print(F("GMT zone: "));Serial.println(GMT_zone);
      Serial.print(F("NTP_data: "));Serial.println(NTP_data);
      Serial.print(F("NTP_dataLight: "));Serial.println(NTP_dataLight);
      */

      Serial.println(F("NTP: OK to obtain time"));
      Serial.println(F("RTC: OK setup ESP32 RTC"));
      RTC_status = true;
      return true;
    }

    if(!getLocalTime(&timeinfo)){
      Serial.println(F("NTP: Failed to obtain time"));
      return false;
    }
  } return false;
}

void OTA_logbook() {

}

void calc_dist(String rxPacket,int pos2) {
  //IU1FIL-14>APLRT1,WIDE1-1,qAO,IR1CH-11                    :!/81)+PV-kk<&QQRX 145.500 Bat=V (mA)
  //IW1CGW-13>APHRM0,WIDE2-2,qAO,IZ1VCX-10                   :!4419.22N/00748.58E_.../...g...t055r...p...P...h89b10104 IW1CGW weather v.12 [1:0]
  //String rxPacket = "IU1FIL-14>APLRT1,WIDE1-1,qAO,IR1CH-11 :!/81)+PV-kk<&QQRX 145.500 Bat=V (mA)";
  //String rxPacket = "IW1CGW-13>APHRM0,WIDE2-2,qAO,IZ1VCX-10:!4419.22N/00748.58E_.../...g...t055r...p...P...h89b10104 IW1CGW weather v.12 [1:0]";
  //IQ2SW-10>APLRG1,TCPIP*,qAC,T2CHILE                       :!L7H1XPh+#a xGLoRa APRS iGATE-DIGI ARI-Saronno
  //IK2XRO-10>APLRG1,IU2SKJ-10*,qAO,IQ4FE-2                  :!L7ScFPp"pa xGLoRa APRS Batt=4.29V
  //IW1QAF-23>APLRW1,WIDE1-1,qAR,IZ1HKE-10                   :!/8%k8PZxo_ xG252/013g021t061r000p002L000h94b09580Experimental LoRa APRS Wx Station
  //IU1FIL-14>APLRT1,WIDE1-1,qAO,IU1LCU-11                   :!/8/gKPW22k@\Q Bat=V (mA)

    float myLat_igate;
    float myLon_igate;
    float LatRx; 
    float LonRx; 

  if ( rxPacket.substring(pos2 +6, pos2+7) == "." && rxPacket.substring(pos2 +16, pos2+17) == "." ) { 
    float LatRx_g = atoi((rxPacket.substring(pos2 +2, pos2+4)).c_str());
    float LatRx_m = atoi((rxPacket.substring(pos2 +4, pos2+6)).c_str());
    float LatRx_s = atoi((rxPacket.substring(pos2 +7, pos2+9)).c_str());
    String LatSign = rxPacket.substring(pos2 +9, pos2+10);
    //Serial.print("LatSign: ");Serial.println(LatSign)

    float LonRx_g = atoi((rxPacket.substring(pos2 +12, pos2+14)).c_str());
    float LonRx_m = atoi((rxPacket.substring(pos2 +14, pos2+16)).c_str());
    float LonRx_s = atoi((rxPacket.substring(pos2 +17, pos2+19)).c_str());
    String LonSign = rxPacket.substring(pos2 +19, pos2+20);
    //Serial.print("LonSign: ");Serial.println(LonSign);

    LatRx = LatRx_g + (LatRx_m/60) + (LatRx_s/6000);
    LonRx = LonRx_g + (LonRx_m/60) + (LonRx_s/6000);
    if ( LatSign == "S" ) LatRx = LatRx * -1;
    if ( LonSign == "W" ) LonRx = LonRx * -1;
  }

  else {
        //const String& GPSPacket         = rxPacket.substring(rxPacket.indexOf(":")+3);
        const String& GPSPacket         = rxPacket.substring(pos2+3);
        
        const String& encodedLatitude   = GPSPacket.substring(0,4);
        const String& encodedLongtitude = GPSPacket.substring(4,8);
        int Y1 = int(encodedLatitude[0]);
        int Y2 = int(encodedLatitude[1]);
        int Y3 = int(encodedLatitude[2]);
        int Y4 = int(encodedLatitude[3]);
        LatRx = 90.0 - ((((Y1-33) * pow(91,3)) + ((Y2-33) * pow(91,2)) + ((Y3-33) * 91) + Y4-33) / 380926.0);
        
        int X1 = int(encodedLongtitude[0]);
        int X2 = int(encodedLongtitude[1]);
        int X3 = int(encodedLongtitude[2]);
        int X4 = int(encodedLongtitude[3]);
        LonRx = -180.0 + ((((X1-33) * pow(91,3)) + ((X2-33) * pow(91,2)) + ((X3-33) * 91) + X4-33) / 190463.0);
  }

    myLat_igate = atof(lat_igate);
    myLon_igate = atof(lon_igate);

  Km = calculateDistance(myLat_igate, myLon_igate, LatRx, LonRx)/1000;

  if ( LatRx == 0 || LonRx == 0 ) Km = 0;

  /*
  Serial.print("LatRx: ");Serial.println(LatRx);
  Serial.print("LonRx: ");Serial.println(LonRx); 
  Serial.print("RX: ");Serial.print(LatRx,8);Serial.print(",");Serial.print(LonRx,8);
  Serial.print(" - da km: ");Serial.println(Km);
  */
}




//**************************************
// * WUNDERGROUND
//
// https://weatherstation.wunderground.com/weatherstation/updateweatherstation.php?ID=IMONDO21&PASSWORD=ywz1K6rv&dateutc=now&humidity=59&action=updateraw
//
//**************************************


void wunder_send() {      // https://support.weather.com/s/article/Managing-Your-Data-Rights?language=en_US&subcategory=Personal_Weather_Stations&type=wu
  if (( BM_sensor_status ) ) {

    //float press = getPressure();
    //press = press*0.02953f; // 1 mbar = 0.02953 inHg

    // (32 °C × 9/5) + 32 = 89,6 °F

   
    String wunder_data;
    wunder_data = "http://weatherstation.wunderground.com/weatherstation/updateweatherstation.php?ID=" + String(wunderid) + "&PASSWORD=" + String(wunderpwd) + "&dateutc=now&tempf=" + String(getTempAPRS()) + "&baromin=" + String(Press*0.02953f);
    
    if (AHTstatus || BMEstatus || DHT22status) wunder_data = wunder_data + "&dewptf=" + String((DewPoint * 9/5) + 32) + "&humidity=" + String(getHumAPRS());

    if ( USE_anemometer)  wunder_data = wunder_data + "&windspeedmph=" + String(windSpeedAPRS(windLongPeriodSpeed)) + "&windgustmph=" + String(windSpeedAPRS(gust));
  
    if ( USE_weathervane ) wunder_data = wunder_data + "&winddir=" + String(getWindDirAPRS());
    
    wunder_data = wunder_data + "&softwaretype=IW1CGW_LoRa_Meteostation&action=updateraw";
    
    //Serial.print(wunder_data);

    /*----------------------------------------------
    //char wunder_data[350]; 
    sprintf(wunder_data, "http://weatherstation.wunderground.com/weatherstation/updateweatherstation.php?ID=%s&PASSWORD=%s&dateutc=now&tempf=%s&dewptf=%s&winddir=%s&windspeedmph=%s&windgustmph=%s&humidity=%s&baromin=%s&softwaretype=IW1CGW_LoRa_Meteostation&action=updateraw",
    wunderid,wunderpwd,String(getTempAPRS()),String((getDewPoint() * 9/5) + 32),String(getWindDirAPRS()),String(windSpeedAPRS(windLongPeriodSpeed)),String(windSpeedAPRS(gust)),String(getHumAPRS()),String(getPressure()*0.02953f));
    */
     
    wunderSendStatus=0;
    HTTPClient wunder;
    wunder.begin(wunder_data);
    delay(500);

    Serial.print(F("beacon WUnder: "));
    int httpCode = wunder.GET();
    if (httpCode == 200) {
      String reply = wunder.getString();
      
      if(reply.indexOf("success") != -1 ) {
        wunderSendStatus=1;
        Serial.println(F("success"));
      }
    }
    else {
      Serial.println("error");
      wunderSendStatus=9;
    }
    wunder.end();

    
  } 
} 

void make_blink() {
  if ( millis() - Pled_millis >= 750) {
    Pled = !Pled;
    esp_task_wdt_reset(); //--- reset al contatore del watchdog
    if ( String(call) == "IW1CBG" ) {
      if (Pled ) digitalWrite(PLed_life_int, HIGH);
      if (!Pled ) digitalWrite(PLed_life_int, LOW);
    } 
    Pled_millis = millis();
  } 
}

String windToWeb(float windValue) {
  if (USE_anemometer) return String(windValue);
  else return "N/A";
}

void zambrettiRoutine() {
  float pressureActual = Press;
  float zFloating;
  
  Serial.print("pressureTrendReference: ");Serial.println(pressureTrendReference);
  Serial.print("pressureActual: ");Serial.println(pressureActual);
 
  pressureTrend = "steady";
  zFloating = 144 - 0.13 * pressureActual;    //--- calcolo per "steady"
      if (zFloating <= 11) weatherSymbol =  "&#9728;";
      else if (zFloating <= 12) weatherSymbol =  "&#9925;";
      else if (zFloating <= 15) weatherSymbol = "&#127782;";
      else if (zFloating <= 18) weatherSymbol =  "&#127783;";
      else if (zFloating > 18) weatherSymbol =  "&#127785;";
      else weatherSymbol =  "&#128269;";

  if (pressureTrendReference + 1.6 < pressureActual) {
    pressureTrend = "rising";
    zFloating = 185 - 0.16 * pressureActual;
    if (zFloating <= 22) weatherSymbol = "&#9728;";
    else if (zFloating <= 25) weatherSymbol = "&#9925;";
    else if (zFloating <= 30) weatherSymbol = "&#127782;";
    else if (zFloating > 30) weatherSymbol = "&#127785;";
    else weatherSymbol = "&#128269;";
  }

  if (pressureTrendReference - 1.6 > pressureActual) {
    pressureTrend = "falling";
    zFloating = 127 - 0.12 * pressureActual;
    if (zFloating <= 2) weatherSymbol = "&#9728;";
    else if (zFloating <= 4) weatherSymbol = "&#9925;";
    else if (zFloating <= 7) weatherSymbol = "&#127782;";
    else if (zFloating > 7) weatherSymbol = "&#127783;";
    else weatherSymbol = "&#128269;";
  }

  forecast = "";
  if ( pressureTrend == "falling" ) forecast = "peggioramento, ";
  if ( pressureTrend == "rising" ) forecast = "miglioramento, ";
  if (zFloating > 0 && zFloating <= 4) forecast += "sereno";
  if (zFloating >4 && zFloating <= 5) forecast += "nubi sparse";
  if (zFloating >5 && zFloating <= 8) forecast += "instabilita, pioggia";
  if (zFloating >8 && zFloating <= 9) forecast += "molto instabile, pioggia";

  if (zFloating > 9 && zFloating <= 12) forecast += "sereno";
  if (zFloating > 12 && zFloating <= 13) forecast += "nuvolosita in aumento";
  if (zFloating > 13 && zFloating <= 14) forecast += "nuvolosita brevi pioggie e schiarite";
  if (zFloating > 14 && zFloating <= 15) forecast += "nuvoloso, instabile, possibilità piovaschi";
  if (zFloating > 15 && zFloating <= 16) forecast += "nuvoloso, instabile, piovaschi";
  if (zFloating > 16 && zFloating <= 17) forecast += "pioggia a intervalli frequenti";
  if (zFloating > 17 && zFloating <= 18) forecast += "molto instabile pioggia";
  if (zFloating > 18 && zFloating <= 19) forecast += "pioggia intensa";
  if (zFloating > 19 && zFloating <= 20) forecast += "";
  if (zFloating > 20 && zFloating <= 22) forecast += "sereno";
  if (zFloating > 22 && zFloating <= 23) forecast += "poche nubi";
  if (zFloating > 23 && zFloating <= 25) forecast += "nubi, possibili scrosci";
  if (zFloating > 25 && zFloating <= 26) forecast += "nuvolovo, instabilità";
  if (zFloating > 26 && zFloating <= 27) forecast += "piuttosto nuvoloso";
  if (zFloating > 27 && zFloating <= 28) forecast += "nuvoloso, instabile";
  if (zFloating > 28 && zFloating <= 29) forecast += "nuvoloso, instabile";
  if (zFloating > 29 && zFloating <= 30) forecast += "molto nuvoloso, instabile";
  if (zFloating > 30 && zFloating <= 31) forecast += "temporali";
  if (zFloating >31 ) forecast += "forti temporali";

  /*
  if (weatherSymbol == "&#128269;" ) forecast = "";
  if (weatherSymbol == "&#9728;" ) forecast = "bel tempo";
  if (weatherSymbol == "&#9925;" ) forecast = "nuvoloso";
  if (weatherSymbol == "&#127782;" ) forecast = "peggioramento, pioggia";
  if (weatherSymbol == "&#127783;" ) forecast = "maltempo, pioggia";
  if (weatherSymbol == "&#127785;" ) forecast = "maltempo, temporali";
  */

  //Serial.print(F("trend: "));Serial.println(pressureTrend );
  //Serial.print(F("valore di zFloating: "));Serial.println( zFloating );
  //Serial.print(F("weatherSymbol: "));Serial.println(weatherSymbol);

  pressureTrendReference = pressureActual;
  pressureTrendTimeout = millis();

}

  bool check_pwd() {                             //--- check
    dashboard_activity_millis = millis();
    //Serial.println(".. RESET ..");
    if ( String(dash_pwd) == String(tmp_dash_pwd) )
    return true;
    else
    return false;   
  }

  void start_sleep() { 
    LoRa.sleep();
    Serial.println("LoRa RTX sleep");
    delay(350);
    delay(350);
    WiFi.disconnect(true); // Disabilita WiFi e cancella credenziali salvate
    delay(350);
    WiFi.mode(WIFI_OFF);   // Spegne il modulo WiFi
    Serial.println("WiFi module sleep");
    /*
    il semplice sleep del TTGO porta il consumo in sleep a circa 5 mA
    spegnendo tutti i sensori e il chip RTX LoRa il consumo scende ulteriormente a soli 1.8 mA/h    
    lo spegnimento del display OLED anche nel normale funzionamento non fa scender ein modo significativo il consumo che é di circa 
    153 mA per il funzionamento in stand-by che salgono a 260 mA durante la trasmissione al più alto livello di potenza
    */


    
    if ( BMPstatus ) { 
      bmp.setSampling(Adafruit_BMP280::MODE_SLEEP);
      Serial.println("BMP280 sleep");
    }

    if ( AHTstatus ) {
      Wire.beginTransmission(0x38);
      Wire.write(0xB0);  // Comando per entrare in sospensione
      Wire.endTransmission();
      usleep(2000);  // Attendi almeno 1 ms per assicurarsi che il comando sia processato
      Serial.println("AHT20 sleep");
  }

    if ( BMEstatus ) { 
      Wire.beginTransmission(0x76);
      Wire.write(0xF4);  // Registro di controllo
      Wire.write(0x00);  // Modalità sospensione (sleep mode)
      Wire.endTransmission();
      usleep(2000);  // Attendi almeno 1 ms per assicurarsi che il comando sia processato
      Serial.println("BME280 sleep");
    }

    if ( INA226_status ) { 
      Wire.beginTransmission(0x40);
      Wire.write(0x00);
      Wire.write(0x00);  // Byte alto (MSB)
      Wire.write(0x00);  // Byte basso (LSB)
      Wire.endTransmission();
      Serial.println("INA226 sleep");
    }
 
    display.dim(true);
    display.display();

    
    // First we configure the wake up source We set our ESP32 to wake up every 5 seconds
    //esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    
    esp_sleep_enable_timer_wakeup(tx_interval * 60 * uS_TO_S_FACTOR);
    //esp_sleep_enable_timer_wakeup(30 * uS_TO_S_FACTOR);
    
    Serial.println("Setup ESP32 to sleep for every " + String(tx_interval * 60) + " Seconds");


    Serial.println("Going to deep sleep now");
    delay(1000);
    Serial.flush(); 
    esp_deep_sleep_start();
    Serial.println("This will never be printed");
  }


  /*
  void print_wakeup_reason(){
    esp_sleep_wakeup_cause_t wakeup_reason;
  
    wakeup_reason = esp_sleep_get_wakeup_cause();

    switch(wakeup_reason)
    {
      case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
      case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
      case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
      case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
      case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
      default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
    }

  }
  */




String generateEncodedTelemetryBytes(int tempValue) { 
  //String encodedBytes;
  //int tempValue;

  int firstByte   = tempValue / 91;
  tempValue       -= firstByte * 91;

  encodedBytes    = char(firstByte + 33);
  encodedBytes    += char(tempValue + 33);
  //Serial.print("encodedBytes: ");Serial.println(encodedBytes);
  return encodedBytes;
}


// --- attiva il backup dell' iGate per fault WiFi o APRS-FI
void start_Backup() {
  Serial.println(F(".. backup igate/digi actived.."));
    if ( igateSwitch == HIGH ) {
      digiSwitch = HIGH;
      igateSwitch = LOW;
    }
    if ( meteo_tx_mode == 2 ) meteo_tx_mode = 1;
    backupStatus = true;
    aprs_login = false;
  }
// --- attiva il backup dell' iGate per fault WiFi o APRS-FI


// --- ripristina il setup ante esecuzione del backup
void stop_Backup() { 
  Serial.println(F(".. backup igate/digi deactivated .."));
  meteo_tx_mode = 0;
  digiSwitch = LOW;
  igateSwitch = LOW;
  meteo_tx_mode = EEPROM.read( 164 );               //--- ripristinas il setup ante esecuzione del backupricarica i valori previsti
  if ( EEPROM.read( 166 ) > 0 ) igateSwitch = true;  //--- Use_IGATE
  if ( EEPROM.read( 167 ) > 0 ) digiSwitch = true;   //--- USE_DIGIPEATER
  verifica_parametri();                              //--- verifica coerenza iGate/digi etc
  backupStatus = false;
  aprs_login = false;
}
// --- ripristinas il setup ante esecuzione del backup




void lora_setup() {
  
  SPI.begin(LoRa_SCK, LoRa_MISO, LoRa_MOSI, LoRa_SS);
  LoRa.setPins(LoRa_SS, LoRa_RST, LoRa_DIO0);
  if (!LoRa.begin(atoi(frequencyC)*1000)) {
    Serial.println("Failed to setup LoRa module.");
    while (1);
  }

  if ( mod_type != 212 ) {  
    LoRa.setSignalBandwidth(LoRa_SignalBandwidth);
    LoRa.setSpreadingFactor(LoRa_SpreadingFactor);    // --- parametri std
    LoRa.setCodingRate4(LoRa_CodingRate4);
  }

  if ( mod_type == 212 ) {  
    LoRa.setSignalBandwidth(LoRa_SignalBandwidth);
    LoRa.setSpreadingFactor(LoRa_XSpreadingFactor_poland);    // --- parametri poland
    LoRa.setCodingRate4(LoRa_XCodingRate4_poland);
  }

  LoRa.enableCrc();
  //LoRa.setGain(LoRa_rx_gain);
  // delay(1000);
  LoRa.setTxPower(LoRa_power);
  delay(1000);
}


void lora_send(String tx_data) {
  digitalWrite(PLed_life_int, HIGH);
  Serial.println("TX: " + tx_data.substring(0,(tx_data.length()-1)));
  //Serial.println("TX: " + tx_data);
  //Serial.println("\nTX: " + tx_data.substring(0,(tx_data.length()-1)));
  //LoRa.setFrequency(atoi(frequencyC)*1000);
  LoRa.beginPacket();
  LoRa.write('<');
  LoRa.write(0xFF);
  LoRa.write(0x01);
  
  LoRa.write((const uint8_t *)tx_data.c_str(), tx_data.length());
  LoRa.endPacket();

  millis_token_tx = millis();
  token_tx = false;
  digitalWrite(PLed_life_int, LOW);
}




void setup_WiFi( String WiFi_ssiD, String WiFi_pwd ) {

    WiFi.setAutoReconnect(true);
    WiFi.begin(WiFi_ssiD, WiFi_pwd);      // Connessione alla rete WiFi esistente (Station) se non si é in sleep mode
    Serial.print(F("Connecting to WiFi: " ));Serial.print(WiFi_ssiD);
    retr = 0 ;
    while (retr <= 20 ) {
      retr++;
      Serial.print(F("."));
      delay(1000);
      if ( WiFi.status() == WL_CONNECTED ) retr = 21;
    }

    if ( WiFi.status() == WL_CONNECTED ) {
      Serial.print(F("\nConnected, IP: "));
      Serial.println(WiFi.localIP());
      myIP = ipToString(WiFi.localIP());
    } 
    else {
      Serial.println(F("\nerror connecting to wifi "));
      myIP = "0.0.0.0";
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
40 - 40 // set tx_interval 1-30 | 10
41 - 46 // frequenza | 6 caratteri ex: 433775
47 - 51 // drift termometro | 5 caratteri
52 - 52 // zambretti si/no
53 - 57 // aprs passcode
58 - 58 // *** CONTROLLO FAMIGLIA *** | value 102 -- questo progetto ha come controllo 102
59 - 59 // gmt time | ex: +1 - base +12 
60 - 101 // info meteo: .. WXmeteo Alma Frabosa - 650m .. | 40 caratteri + flag fine testo - carattere cella 101 -
102 - 111 // digi_route | 10 caratteri - ex: iw1cgw-10
112 - 152 // info igate: .. WXmeteo Alma Frabosa - 650m .. | 40 caratteri + flag fine testo - carattere cella 152 -
153 - 162 // digi_banned | 10 caratteri - ex: iw1cgw-10
163 - 163 // max_digi_radius | da 1 a 50 Km - default 0 - disabled
164 - 164 // meteo_tx_mode | 0 = disable - 1= enable ( via igate if WiFi OK - via RF if WiFi  KO )
165 - 165 // switch backup for fault igate [backupStatusSwitch]
166 - 166 / /switch igate [igateSwitch]
167 - 167 // switch digipeater [digiSWitch]
168 - 168 // display on/off
169 - 169 // drift pressure

170 - 189 // WiFi ssiD [ 20 caratteri ]
190 - 214 // WiFi password [ 25 caratteri ]

215 - 234 // - VUOTO - ex APRS_server *** pulito dalla 2024 09 xx

235 - 244 // latitude igate | 10 caratteri
245 - 255 // longitudine igate | 11 caratteri 

256 - 265 // Wunder id | 10 caratteri
266 - 275 // Wunder station key | 10 caratteri
276 - 276 // Wunder use | 0/1

277 - 277    // send iGate beacon also in RF

278 - 283 // min_temp | 6 caratteri - ex: -13.23
284 - 289 // max temp | 6 caratteri - ex: +41.32
290 - 296 // min press | 7 caratteri - ex: 1000.6
297 - 303 // max press | 7 caratteri - ex: 1200.9

304       // telemSwitch || default off == 0
305       // USE_V12 || default off == 0
306 - 310 // cm percorsi dal magnete della girella | 5 caratteri - ex 14.55
311 - 315 // drift_batteryC || +/- 2 volt - 5 caratteri - ex: +1.32
316       // drift_weathervane || +/- 125
317 - 336 // AP_pwd [ 20 caratteri ]

337       // A_SCALE | modalità Amperometro: 0 scala per amperometri con resistenza shunt standard R100| 1 scala per amperometri con resistenza shunt R010

338       // OTA_update on/off || default on = 1 or any value != '212'

339 - 346 // dash_pwd | password per il setup 8 caratteri
347       // verify is dash_pwd is set | 212 arbitrary value only for value set 
348       // mod_type | if 212 arbitrary value set for polish system also default value
349       // hide_server_aprs if value is 212 arbitrary value 
350       // cnt_telem || contatore dei pacchetti di telemetria
351       // switch_sleep | if 212 arbitrary value for set switch_sleep

352       // autoreboot on/off | default on = 1
353       // AP_auto_shutdown on/ff | default off = 0

360 - 379 // WiFi2 ssiD [ 20 caratteri ]
380 - 424 // WiFi2 password [ 45 caratteri ]
*/




/*
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


RAM:   [=         ]  14.8% (used 48588 bytes from 327680 bytes)
Flash: [========= ]  85.4% (used 1119861 bytes from 1310720 bytes)

RAM:   [=         ]  14.8% (used 48596 bytes from 327680 bytes)
Flash: [========= ]  85.5% (used 1120753 bytes from 1310720 bytes)

RAM:   [==        ]  15.5% (used 50812 bytes from 327680 bytes)
Flash: [========= ]  89.6% (used 1174245 bytes from 1310720 bytes)

RAM:   [==        ]  15.5% (used 50812 bytes from 327680 bytes)
Flash: [========= ]  89.4% (used 1172253 bytes from 1310720 bytes) 

RAM:   [==        ]  15.9% (used 51948 bytes from 327680 bytes)
Flash: [========= ]  90.1% (used 1181441 bytes from 1310720 bytes) 

RAM:   [==        ]  16.0% (used 52332 bytes from 327680 bytes)
Flash: [========= ]  90.6% (used 1188053 bytes from 1310720 bytes)  

RAM:   [==        ]  16.0% (used 52460 bytes from 327680 bytes)
Flash: [========= ]  91.1% (used 1193969 bytes from 1310720 bytes)

RAM:   [==        ]  16.0% (used 52332 bytes from 327680 bytes)
Flash: [========= ]  90.7% (used 1189017 bytes from 1310720 bytes)

RAM:   [==        ]  16.0% (used 52340 bytes from 327680 bytes)
Flash: [========= ]  90.7% (used 1189073 bytes from 1310720 bytes) 

RAM:   [==        ]  16.0% (used 52436 bytes from 327680 bytes)
Flash: [========= ]  91.0% (used 1192237 bytes from 1310720 bytes)

RAM:   [==        ]  16.0% (used 52444 bytes from 327680 bytes)
Flash: [========= ]  90.9% (used 1191469 bytes from 1310720 bytes)

RAM:   [==        ]  16.0% (used 52436 bytes from 327680 bytes)
Flash: [========= ]  90.9% (used 1192045 bytes from 1310720 bytes)
----------------------------------------------------------------------- // libreria ham_lib
RAM:   [==        ]  16.3% (used 53372 bytes from 327680 bytes)
Flash: [========= ]  93.3% (used 1222345 bytes from 1310720 bytes)
----------------------------------------------------------------------- // ritorno a libreria tradizionale
RAM:   [==        ]  16.0% (used 52428 bytes from 327680 bytes)
Flash: [========= ]  91.0% (used 1192961 bytes from 1310720 bytes)





*/

