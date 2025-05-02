// GENERAL SETTINGS

#define Control "1.02"                  // NON VARIARE CODICE CONTROLLO FAMIGLIA 
#define Project "LoRa_APRS_iw1cgw"

#define Release "2.01"

char Build[50] = "250502";
#define Hostname "LoRa_APRS_iw1cgw"         // Hostname, name of computer visible in the network


#define telem_param_0 ":EQNS.0,1,0,0,1,0,0,0.0196,0,0,0.1,0,0,0.01,-1.27"   // !!! riportare gli esatti valori di parametrizzazione dei sensori !!!!
#define telem_param_1 ":UNIT.Pkt/10m,Pkt/10m,Volt,Volt,Ampere"
#define telem_param_2 ":PARM.RxDigi,TxDigi,Batt,Volt Aux,Ampere Aux"       // telemetria se modulo INA226 montato
#define telem_param_3 ":BITS.00000000,Telemetry by IW1CGW"

#define telem_meteo_param_0 ":EQNS.0,0.0196,0,0,0.1,0,0,0.01,-1.27"   // !!! riportare gli esatti valori di parametrizzazione dei sensori !!!!
#define telem_meteo_param_1 ":UNIT.Volt,Volt,Ampere"
#define telem_meteo_param_2 ":PARM.Batt,Volt Aux,Ampere Aux"       // telemetria se modulo INA226 montato

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
//#define TIME_TO_SLEEP  30        /* Time ESP32 will go to sleep (in seconds) */

#define EEPROM_SIZE  425 // // EEPROM size puo' indirizzare da 0 a 425

#define AUTORESTART_millis 259200   // 72 ore - valore in secondi
#define Timeout_AP_millis 300000    // 5 minuti - valore in millisecondi

//#define WiFi_retry_login_millis     60000  // 1 minuti - valore in millisecondi
//#define APRS_retry_login_millis     30000  // 30 secondi - valore in millisecondi
#define WiFi_lost_time 120000                // 120 secondi - valore in millisecondi


// la GPIO 34 in modalità 'contampulsi' fuori da ciclo CPU é occupata da anemometro - https://forum.arduino.cc/t/solved-esp32-anemometer-on-reed-switch/1090185/11
#define VOLTAGE_SENSOR_PIN 35       // use only ADC_5 pins
#define weathervane_PIN 36          // use only ADC_1 pins
#define V12_SENSOR_PIN 39           // use only ADC_2 pins

#define amperometer_param_b 0.01
#define amperometer_param_c -1.27
#define voltmeter_param_b 0.1
#define voltmeter_param_c 25.5

#define PLed_life_int 25            // pin number for LED - 25 is internal led green
#define PLed_life 12                // pin number for LED [ ok 12  ]

#define DHT22_PIN 13                // pin number for DTH22 [  ]

#define DHT22_NUM_SAMPLES 10      // Number of temperature and humidity samples for average calculation
#define sensor_refresh_slow 3000; // for use whith DHT22
#define sensor_refresh_fast  500; // for use whith other sensor

#define GRAPH_LIMIT 144                   // how many values to store for graphs (too high can cause errors)
//#define ANEMOMETER_LENGTH 0.14          // how long distance (meters) is done by spinning anemometer from one magnet to the next onehow long distance (meters) is done by spinning anemometer from one magnet to the next one
#define ANEMO_RECALC_LIMIT 2              // calibrate hall sensor - 2 giri per considerare letture

#define ANEMO_RECALC_LIMIT_TIMEOUT 120    // auto update long-period speed after x seconds (should be METEO_BEACON * 60)
#define ANEMO_RECALC_ACTUAL_SPEED 4       // set actual wind speed to 0 if anemometer is not spinning for x seconds

#define APRS_IS_Port 14580                  // server port, keep default (14580)


#define USE_LASTRX_STATUS true             // display Last RX status on igate

#define SERIAL_BAUD 115200                 // serial baud

#define LoRa_SCK 5
#define LoRa_MISO 19
#define LoRa_MOSI 27
#define LoRa_SS 18
#define LoRa_RST 14
#define LoRa_DIO0 26

//-------------------------------------------// parametri OESTYLE standard - standard 300 baud
#define LoRa_SpreadingFactor 12         // The Spreading Factor varies between 128 = 2^7 (SF7) and 4096 = 2^12 (SF12)
#define LoRa_SignalBandwidth 125000
#define LoRa_CodingRate4 5
#define LoRa_type "BW 125KHz CR 4:5 SF 12 (300bps)"
/*
set 5 for 4:5
set 6 for 4:6
set 7 for 4:7
set 8 for 4:8
*/

//-------------------------------------------// parametri polacchi, ma vedi anche; https://meshtastic.org/docs/overview/radio-settings
#define LoRa_XSpreadingFactor_poland 9           
#define LoRa_XCodingRate4_poland 7
#define LoRa_type_poland "BW 125KHz CR 4:7 SF 9 (1200bps)"


// DEBUG SETTINGS
#define INA226_DEBUG false                // enable debug sensor INA226
#define ANEMO_DEBUG_MODE true              // enable debug mode for hall sensor
#define weathervane_DEBUG_MODE false
#define HTTP_DEBUG_MODE false              // print incoming HTTP requests
// EXPERIMENTAL DEBUG ONLY SETTINGS
#define DIGI_IGNORE_PARADIGM false         // digipeat packets regardless if they contain WIDEn-N
#define debug_digi false