#ifndef MAIN_H
#define MAIN_H

#include <SPI.h>
#include <EEPROM.h>
#include <SimpleTimer.h>
#include <SoftwareSerial.h>
#include <Rotary.h>

#include <LibAPRS.h>
#include <KISS.h>

#define ADAFRUIT_NO_BUTTON
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>

#define _GPS_NO_STATS
#include <TinyGPS.h>

// use flags
#define USE_SCREEN
#define USE_RECEIVE
#define USE_RECEIVE_INFO
#define USE_GPS
#define USE_PRECISE_DISTANCE
#define USE_RX_ON_CALLBACK
#define USE_COMPRESSED_POSITION
//#define USE_SERIAL
//#define USE_SMART_BEACONING
//#define USE_KISS

#ifdef USE_KISS
  #undef USE_SCREEN
  #define USE_SERIAL
#endif

// common
#define ADC_REFERENCE REF_5V // _3V3
#define OPEN_SQUELCH false

#define GPS_POLL_DURATION_MS 1000 // for how long to poll gps
#define UPDATE_HEURISTICS_MAP_SIZE 9
#define TIMER_DISABLED -1

#define TX_PREAMBLE 350
#define TX_TAIL 50

// screen
#define SCREEN_TIMEOUT 30000UL // 30 seconds
#define SCREEN_CONTRAST 40
#define SCREEN_UPDATE_PERIOD 3000  // fow how often to update screen
#define GPS_UPDATE_PERIOD 3000  // fow how often to update screen

// aprs symbols
#define APRS_SYM_PHONE '$'
#define APRS_SYM_CAR '>'
#define APRS_SYM_BIKE 'b'
#define APRS_SYM_JEEP 'j'
#define APRS_SYM_JOGGER '['

// aprs ssids
#define APRS_SSID_GENERIC_2 2
#define APRS_SSID_GENERIC_3 3
#define APRS_SSID_WALKIE_TALKIE 7
#define APRS_SSID_PRIMARY_MOBILE 9
#define APRS_SSID_ADDITIONAL 15

// aprs config
#define APRS_MY_CALLSIGN "NOCALL"

// smart beaconing parameters
#define APRS_SB_LOW_SPEED 5
#define APRS_SB_HIGH_SPEED 100
#define APRS_SB_SLOW_RATE 1200
#define APRS_SB_FAST_RATE 60
#define APRS_SB_TURN_MIN 15
#define APRS_SB_TURN_TH 10
#define APRS_SB_TURN_SLOPE 240

// eeprom storage
#define EEPROM_ACTIVE_HEURISTIC_IDX 0
#define EEPROM_ACTIVE_SYMBOL_IDX 1

// pins
#define PIN_SCREEN_LIGHT  8
#define PIN_GPS_RX 2
#define PIN_GPS_TX 15
#define PIN_ROT_CLK 17
#define PIN_ROT_DT 18
#define PIN_ROT_BTN 19

// Screen state
enum t_screen_state {
  S_STATUS = 0,
  S_CONFIG_HEURISTICS,
  S_CONFIG_SYMBOL,
  S_CONFIG_SSID,
  S_RECEIVE
};

// APRS update heuristics
enum t_update_heuristics {
  H_OFF = 0,
  H_PERIODIC_1MIN,
  H_PERIODIC_5MIN,
  H_PERIODIC_10MIN,
  H_PERIODIC_30MIN,
  H_RANGE_500M,
  H_RANGE_1KM,
  H_RANGE_5KM,
  H_SMART_BEACONING,
  H_MAX_HEURISTICS = H_SMART_BEACONING 
};

void setSymbol(char sym);

bool isScreenOn();
void screenOff();
void screenOn();

void deleteActiveAprsUpdateTimer();

void activateAprsUpdateHeuristic(char heuristic);
void heuristicProcessSmartBeaconing();
void heuristicDistanceChanged();

void setAprsUpdateFlag();

char* deg_to_nmea(long deg, boolean is_lat);
char* deg_to_compressed(long deg, boolean is_lat);
char *deg_to_qth(long lat, long lon);

long distanceBetween(long lat1, long long1, long lat2, long long2);
void updateDistance();

char *get_comment();
void aprs_msg_callback(struct AX25Ctx *ctx);

void updateGpsData();
void updateScreen();
void updateScreenAndGps();
void displayPrintPgm(PGM_P str);

bool sendAprsLocationUpdate();
void selectNextSymbol();

void processPacket();

bool processRotary();
void onLongBtnReleased();
void onRotaryLeft();
void onRotaryRight();
void onBtnReleased();

#endif // MAIN_H