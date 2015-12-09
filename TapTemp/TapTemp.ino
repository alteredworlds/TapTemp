//
// TapTemp
//
// Tap Temp monitor
// Developed with [embedXcode](http://embedXcode.weebly.com)
//
// Author 		Tom Gilbert
// 				Tom Gilbert
//
// Date			07/12/2015 10:50
// Version		<#version#>
//
// Copyright	© Tom Gilbert, 2015
// Licence		<#licence#>
//
// See         ReadMe.txt for references
//


// Core library for code-sense - IDE-based
#if defined(WIRING) // Wiring specific
#   include "Wiring.h"
#elif defined(MAPLE_IDE) // Maple specific
#   include "WProgram.h"
#elif defined(MPIDE) // chipKIT specific
#   include "WProgram.h"
#elif defined(DIGISPARK) // Digispark specific
#   include "Arduino.h"
#elif defined(ENERGIA) // LaunchPad specific
#   include "Energia.h"
#elif defined(LITTLEROBOTFRIENDS) // LittleRobotFriends specific
#   include "LRF.h"
#elif defined(MICRODUINO) // Microduino specific
#   include "Arduino.h"
#elif defined(SPARK) || defined(PARTICLE) // Particle / Spark specific
#   include "Arduino.h"
#elif defined(TEENSYDUINO) // Teensy specific
#   include "Arduino.h"
#elif defined(REDBEARLAB) // RedBearLab specific
#   include "Arduino.h"
#elif defined(ESP8266) // ESP8266 specific
#   include "Arduino.h"
#elif defined(ARDUINO) // Arduino 1.0 and 1.5 specific
#   include "Arduino.h"
#else // error
#   error Platform not defined
#endif // end IDE

// Include application, user and local libraries
#include <SPI.h>
#include <SdFat.h>
#include <RBL_nRF8001.h>

#include <Wire.h>
#include <RTCLib.h>


// Prototypes
//==============================================================================
// Error messages stored in flash.
#define error(msg) sd.errorHalt(msg)


// Define variables and constants

// SD chip select pin.  Be sure to disable any other SPI devices such as Enet.
const uint8_t chipSelect = SS;

// Interval between data records in milliseconds.
// The interval must be greater than the maximum SD write latency plus the
// time to acquire and write data to the SD to avoid overrun errors.
// Run the bench example to check the quality of your SD card.
const uint32_t SAMPLE_INTERVAL_MS = 200;

// want at least 1s between analog reads
const uint32_t MIN_ANALOG_READ_INTERVAL_uS = 1000000;

// want at least 1 record per 60 mins to demonstrate 'alive'
const uint32_t MAX_LOG_INTERVAL_uS = 3600 * 1000000;

// only want to record meaningful changes in analog reading
const uint16_t MIN_ANALOG_DELTA = 1;

#define FILE_CURRENT_NAME "current.csv"
//------------------------------------------------------------------------------
// File system object.
SdFat sd;

// Log file.
SdFile file;

// Time in micros for next data record.
uint32_t logTime = 0;

// Time in micros when last sensor read occurred
uint32_t sensorReadTime = 0;

// Real Time Clock
RTC_DS1307 RTC;

//==============================================================================
// User functions.  Edit writeHeader() and logData() for your requirements.

// we have only a single temperature probe right now...
const uint8_t ANALOG_COUNT = 1;

// last measured analog data
uint16_t recordedData[ANALOG_COUNT];

// Write data header.
void writeHeader() {
    file.print(F("unixtime"));
    for (uint8_t i = 0; i < ANALOG_COUNT; i++) {
        file.print(F(",adc"));
        file.print(i, DEC);
    }
    file.println();
}

uint32_t getTime() {
    DateTime now    = RTC.now();
    uint32_t retVal = now.unixtime();
    
    // DEBUG output
    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(' ');
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.print(" (");
    Serial.print(retVal);
    Serial.print(")");
    
    return retVal;
}


// Log a data record if we need to
//    (big enough delta reading OR time interval)
boolean logIfAppropriate() {
    boolean writeData = false;
    if (file.isOpen()) {
        uint16_t data[ANALOG_COUNT];
        
        // Current time in micros
        uint32_t currentTime = micros();
        
        // delta since last sensor reading was taken big enough to measure again?
        if ((currentTime - sensorReadTime) > MIN_ANALOG_READ_INTERVAL_uS) {
            // OK, time to take a reading from the analog sensor(s)
            sensorReadTime = currentTime;
            
            // DEBUG output
            //Serial.println(F("Reading analog sensor(s)..."));
            
            uint8_t i;
            for (i = 0; i < ANALOG_COUNT; i++) {
                // read twice see final comment by tuxdino: http://forum.arduino.cc/index.php?topic=6261.15
                // Arduino analogRead() seems to combine channel selection & read, but 50us delay btwn required
                analogRead(i);              // select correct channel
                delay(5);
                data[i] = analogRead(i);    // we should now get an accurate reading
            }
            // compare with most recent recorded values - if the data has changed sufficiently
            // in any channel we need to write a log record
            uint16_t diff;
            for (i = 0; i < ANALOG_COUNT; i++) {
                diff = recordedData[i] > data[i] ? recordedData[i] - data[i] : data[i] - recordedData[i];
                writeData = (diff > MIN_ANALOG_DELTA);
                if (writeData) {
                    // item at index i has changed enough
                    // => entire record must be written
                    
                    // update the recorded values
                    for (i = 0; i < ANALOG_COUNT; i++) {
                        recordedData[i] = data[i];
                    }
                    
                    // DEBUG output
//                    Serial.print(F("Old value: "));
//                    Serial.print(recordedData[i]);
//                    Serial.print(F("  New value: "));
//                    Serial.println(data[i]);
                    break;
                }
            }
            
            // write to the log if either data changed on this read cycle
            //  OR long time since last write & time to prove 'alive'
            writeData = (writeData || ((currentTime - logTime) >= MAX_LOG_INTERVAL_uS));
            if (writeData) {
                // DEBUG output
                Serial.print(F("  Logging: "));
                
                // write the time derived from RTC
                file.print(getTime());
                
                // DEBUG output
                Serial.print(F(" "));
                
                // Write ADC data to CSV record.
                for (uint8_t i = 0; i < ANALOG_COUNT; i++) {
                    file.write(',');
                    file.print(data[i]);
                    
                    // DEBUG output
                    if (i > 0) {
                        Serial.print(',');
                    }
                    Serial.println(data[i]);
                }
                file.println();
                
                // Force data to SD and update the directory entry to avoid data loss.
                if (!file.sync() || file.getWriteError()) {
                    error(F("write error"));
                } else {
                    // all OK: update last logged time
                    logTime = currentTime;
                }
            }
        }
    }
    return writeData;
}

boolean handleBleCommands() {
    boolean retVal = false;
    while (ble_available()) {
        byte cmd;
        cmd = ble_read();
        
        // DEBUG output
        Serial.print(F("Ble Command received: "));
        Serial.println(cmd);
        
        retVal = true; // we have handled a command
        
        // Parse data here
        switch (cmd) {
            case 'V': // should be query protocol version
            // but falling through to close the file
            case 'C': {
                if (file.isOpen()) {
                    file.close();
                    
                    // DEBUG output
                    Serial.println(F("Closed file"));
                }
            }
            break;
        }
    }
    return retVal;
}


// Add setup code
void setup() {
    Serial.begin(9600);
    while (!Serial) {} // wait for Leonardo
    delay(1000);
    
    Wire.begin();
    RTC.begin();
    if (!RTC.isrunning()) {
        Serial.println(F("RTC is NOT running!"));
        // following line sets the RTC to the date & time this sketch was compiled
        // uncomment it & upload to set the time, date and start run the RTC!
        RTC.adjust(DateTime(__DATE__, __TIME__));
    }

    // Initialize the SD card at SPI_HALF_SPEED to avoid bus errors with
    // breadboards.  use SPI_FULL_SPEED for better performance.
    if (!sd.begin(chipSelect, SPI_HALF_SPEED)) {
        sd.initErrorHalt();
    }
    
    // Always write to current.csv, but may need to roll
    if (!file.open(FILE_CURRENT_NAME, O_CREAT | O_APPEND | O_WRITE)) {
        error(F("file.open"));
    }
    do {
        delay(10);
    } while (Serial.read() >= 0);
    
    Serial.print(F("Logging to: "));
    Serial.println(FILE_CURRENT_NAME);
    
    // Write data header.
    writeHeader();
    
    // Set BLE Shield name here, max. length 10
    ble_set_name("AW_TAP_01");

    // Init. and start BLE library.
    ble_begin();
    
    // set data to zero (necessary?)
    for (uint16_t i = 0; i < ANALOG_COUNT; i++) {
        recordedData[i] = 0;
    }
    
    // last logged time set 0 => initial time based write
    logTime = 0;
}

void loop() {
    // we want to try handlingBleCommands if we didn't log in this loop
    handleBleCommands() || logIfAppropriate();
    
    //handleBleCommands();
    
    // in any case, need to process outstanding BLE events (e.g.: send data)
    ble_do_events();
}