// GENERAL SETTINGS

#define Project "LoRa_APRS_iw1cgw"
#define Release "1.02"
char Build[7] = "240302";
#define Hostname "LoRa_APRS_iw1cgw"         // Hostname, name of computer visible in the network

/*
 - tocall: APLHI?
   vendor: Giovanni, IW1CGW
   model: LoRa IGate/Digipeater/Telemetry
   class: digi
   contact: iw1cgw@libero.it

 - tocall: APLHM?
   vendor: Giovanni, IW1CGW
   model: LoRa Meteostation
   class: wx
   contact: iw1cgw@libero.it

RAM:   [=         ]  14.8% (used 48556 bytes from 327680 bytes)
Flash: [========  ]  84.7% (used 1109585 bytes from 1310720 bytes)

RAM:   [=         ]  14.8% (used 48588 bytes from 327680 bytes)
Flash: [========= ]  85.4% (used 1119861 bytes from 1310720 bytes)

RAM:   [=         ]  14.8% (used 48572 bytes from 327680 bytes)
Flash: [========= ]  85.3% (used 1117665 bytes from 1310720 bytes)

*/


char OTA_server[15] ="ftpupload.net";       // verificare che l'array sia n+1 carattere del valore
char OTA_user[15] = "if0_35994920";         // verificare che l'array sia n+1 carattere del valore
char OTA_pass[15] = "HaAQ8pdOzLMtWb";       // verificare che l'array sia n+1 carattere del valore

#define OTA_dirName "/htdocs/"
#define OTA_fileName_logbook "data.txt"        // file testo che contiene la progressione dei codici OTA
#define OTA_fileName_ver "DSCN0001.JPG"       // file testo che contiene unicamente il valore della Build del file di upload
#define OTA_fileName_update "DSCN0002.JPG"     // il file binario di upload
#define OTA_web_server "http://iv3sgb.000.pe/"





#define GRAPH_LIMIT 144                   // how many values to store for graphs (too high can cause errors)
#define USE_ANEMOMETER false              // turn on/off wind meter
#define HALL_SENSOR_PIN 35                // use only ADC_1 pins
#define ANEMOMETER_LENGTH 0.25            // how long distance (meters) is done by spinning anemometer from one magnet to the next one
#define ANEMO_RECALC_LIMIT 2              // calibrate hall sensor
#define ANEMO_AC_THRESHOLD 1830           // analog value threshold to trigger magnet detection
#define ANEMO_AC_LOSE      2000           // analog value threshold - magnet is away
#define ANEMO_RECALC_LIMIT_TIMEOUT 900    // auto update long-period speed after x seconds (should be METEO_BEACON * 60)
#define ANEMO_RECALC_ACTUAL_SPEED 4       // set actual wind speed to 0 if anemometer is not spinning for x seconds

#define APRS_IS_Port 14580                  // server port, keep default (14580)

#define USE_LASTRX_STATUS true             // display Last RX status on igate

#define SERIAL_BAUD 115200                           // serial baud

#define LoRa_SCK 5
#define LoRa_MISO 19
#define LoRa_MOSI 27
#define LoRa_SS 18
#define LoRa_RST 14
#define LoRa_DIO0 26
//-------------------------------------------// parametri OESTYLE standard
#define LoRa_SpreadingFactor 12         // The Spreading Factor varies between 128 = 2^7 (SF7) and 4096 = 2^12 (SF12)
#define LoRa_SignalBandwidth 125000
#define LoRa_CodingRate4 5


/*
//-------------------------------------------// parametri sperimentali veloci - https://meshtastic.org/docs/overview/radio-settings
#define LoRa_XSpreadingFactor 9             // Medium Fast / - 143 dB budget - new 
#define LoRa_XSignalBandwidth 250000
#define LoRa_XCodingRate4 4
*/


// DEBUG SETTINGS

#define ANEMO_DEBUG_MODE false                        // enable debug mode for hall sensor
#define HTTP_DEBUG_MODE false                          // print incoming HTTP requests
// EXPERIMENTAL DEBUG ONLY SETTINGS
#define DIGI_IGNORE_PARADIGM false       // digipeat packets regardless if they contain WIDEn-N
