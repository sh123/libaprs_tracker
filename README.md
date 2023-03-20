# aprs_tracker
Tiny standalone single sketch Arduino APRS tracker based on modified version of LibAPRS with external GPS module

Arduino APRS Tracker
====================

Introduction
------------
Arudino APRS tracker is based on Arduino UNO with next periherals: 
 * Nokia screen, PCD8544 - https://www.sparkfun.com/datasheets/LCD/Monochrome/Nokia5110.pdf
 * Rotary encoder - https://www.sparkfun.com/products/15140
 * Micromodem from marqvist - http://unsigned.io/micromodem/
 * GPS receiver, SkyLabs SKM53 - http://www.skylabmodule.com/gpsmodule-antenna-skm53/

Requirements
------------
 * Minimum: Arduino UNO with 2k SRAM, 32k FLASH
   * may require some manual libraries optimization, such as buffer size decrease
   * or different screen, PCD8544 requires 500 bytes of RAM for the buffer
 * Recommended: Arduino MEGA with 8k SRAM
   * NB! needs some tweaks in LibAPRS internals

Building
------------
Use platformio to build and deploy.

Supported user operations
-------------------------
 * Rotary encoder rotation iterates over transmit heuristics selection, such as periodic transmit every 1 minute, 2 minutes, 500 meters, etc.
 * Short encoder release activates current heuristic if it is different from the current one or iterates over different APRS SSIDs
 * Long encoder release forces current position to be sent out
