// GENERAL SETTINGS

#define Project "LoRa_APRS_Igate+Meteo"
char Release[7]= "1.02";
#define Build "240114"
#define DESTCALL "APLGM5"            // NOT MODIFY !!! 
bool Use_WiFi = true;                // enable Wi-Fi connection and HTTP web server
#define Hostname "LORAMETEO"         // Hostname, name of computer visible in the network

// METEO SERVICE SETTINGS


// --- i comment sulle stringhe senza georeferenziazione --- //
// per iGate viene indicato il valore dell'ultimo ascolto
// per meteo viene indicato il banner di GitHub
//#define METEO_STATUS "https://iw1cgw.wordpress.com/"  
#define METEO_STATUS "--- test meteo status ---" 
//#define METEO_STATUS "LoRa Meteo based by OK2DDS project"

//#define igate_info "433.775MHz LoRa iGate/digipeater"

#define USE_METEO_STATUS true           // send status below in timeout of igate packet (needs wifi, igate on and aprs-is)
#define METEO_STATUS_SEND_INTERVAL 10   // [ cgw ] send beacon meteo status after nr. xx beacon meteo

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

#define USE_LASTRX_STATUS true              // display Last RX status on igate

// HTTP GET DATA UPLOAD SERVICE SETTINGS (experimental, for development)

#define Use_UPLOAD false
#define SERVER_URL "http://meteo.mywebsite.com/update.php?values=" // can be used for working with your SQL database
#define UPLOAD_TIMEOUT 5

// LORA MODULE SETTINGS (keep default unless experimental setup)

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


#define Experimental false
//-------------------------------------------// parametri sperimentali veloci - https://meshtastic.org/docs/overview/radio-settings
#define LoRa_XSpreadingFactor 9             // Medium Fast / - 143 dB budget - new 
#define LoRa_XSignalBandwidth 250000
#define LoRa_XCodingRate4 4



// DEBUG SETTINGS

#define ANEMO_DEBUG_MODE false                        // enable debug mode for hall sensor
#define HTTP_DEBUG_MODE false                          // print incoming HTTP requests
// EXPERIMENTAL DEBUG ONLY SETTINGS
#define DIGI_IGNORE_PARADIGM false       // digipeat packets regardless if they contain WIDEn-N
