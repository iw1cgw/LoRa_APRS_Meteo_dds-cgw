// GENERAL SETTINGS

#define Project "LoRa_APRS_iw1cgw"
#define Release "1.02"
char Build[7] = "240411";
#define Hostname "LoRa_APRS_iw1cgw"         // Hostname, name of computer visible in the network

#define AUTORESTART_millis 259200   // 72 ore
#define VOLTAGE_SENSOR_PIN 35                 // use only ADC_1 pins

#define GRAPH_LIMIT 144                   // how many values to store for graphs (too high can cause errors)
#define USE_ANEMOMETER false              // turn on/off wind meter
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
