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


// Prototypes
//==============================================================================
// Error messages stored in flash.
#define error(msg) sd.errorHalt(F(msg))


// Define variables and constants

// SD chip select pin.  Be sure to disable any other SPI devices such as Enet.
const uint8_t chipSelect = SS;

// Interval between data records in milliseconds.
// The interval must be greater than the maximum SD write latency plus the
// time to acquire and write data to the SD to avoid overrun errors.
// Run the bench example to check the quality of your SD card.
const uint32_t SAMPLE_INTERVAL_MS = 200;

// want a minimum logging rate of 1 record per 60 mins to demonstrate 'alive'
const uint32_t MIN_LOG_INTERVAL_uS = 3600 * 1000000;

// only want to record meaningful changes in analog reading
const uint16_t MIN_ANALOG_DELTA = 5;

boolean fileOpen = false;

// Log file base name.  Must be six characters or less.
#define FILE_BASE_NAME "Data"
//------------------------------------------------------------------------------
// File system object.
SdFat sd;

// Log file.
SdFile file;

// Time in micros for next data record.
uint32_t logTime;

//==============================================================================
// User functions.  Edit writeHeader() and logData() for your requirements.

// we have only a single termperature probe right now...
const uint8_t ANALOG_COUNT = 1;
uint16_t recordedData[ANALOG_COUNT];

// Write data header.
void writeHeader() {
    file.print(F("micros"));
    for (uint8_t i = 0; i < ANALOG_COUNT; i++) {
        file.print(F(",adc"));
        file.print(i, DEC);
    }
    file.println();
}

// Log a data record if we need to
//    (big enough delta reading OR time interval)
void logIfAppropriate() {
    if (!file.isOpen()) {
        return;
    }
    uint16_t data[ANALOG_COUNT];
    
    // Current time in micros
    uint32_t currentTime = micros();
    
    // Read all channels in one go
    uint8_t i;
    for (i = 0; i < ANALOG_COUNT; i++) {
        data[i] = analogRead(i);
    }
    // compare with most recent recorded values - if the data has changed sufficiently
    // in any channel we need to write a log record
    uint16_t diff;
    boolean shouldWrite = false;
    for (i = 0; i < ANALOG_COUNT; i++) {
        diff = recordedData[i] > data[i] ? recordedData[i] - data[i] : data[i] - recordedData[i];
        shouldWrite = (diff >= MIN_ANALOG_DELTA);
        if (shouldWrite) {
            // item at index i has changed enough
            // => entire record must be written
            
            // DEBUG output
            Serial.print(F("Old value: "));
            Serial.print(recordedData[i]);
            Serial.print(F("  New value: "));
            Serial.print(data[i]);
            break;
        }
    }
    // if no change, see if last write was too long ago...
    shouldWrite = shouldWrite || ((currentTime - logTime) >= MIN_LOG_INTERVAL_uS);
    if (shouldWrite) {
        // update the recorded values
        for (i = 0; i < ANALOG_COUNT; i++) {
            recordedData[i] = data[i];
        }
        
        // write this time
        file.print(currentTime);
        
        // DEBUG output
        Serial.print(F("  Logged: "));
        
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
            error("write error");
        } else {
            // all OK: update last logged time
            logTime = currentTime;
        }
    }
}

void handleBleCommands() {
    while (ble_available()) {
        byte cmd;
        cmd = ble_read();
        
        // DEBUG output
        Serial.print(F("Ble Command received: "));
        Serial.println(cmd);
        
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
}

// Add setup code
void setup()
{
    const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
    char fileName[13] = FILE_BASE_NAME "00.csv";
    
    Serial.begin(9600);
    while (!Serial) {} // wait for Leonardo
    delay(1000);

    // Initialize the SD card at SPI_HALF_SPEED to avoid bus errors with
    // breadboards.  use SPI_FULL_SPEED for better performance.
    if (!sd.begin(chipSelect, SPI_HALF_SPEED)) {
        sd.initErrorHalt();
    }
    
    // Find an unused file name.
    if (BASE_NAME_SIZE > 6) {
        error("FILE_BASE_NAME too long");
    }
    while (sd.exists(fileName)) {
        if (fileName[BASE_NAME_SIZE + 1] != '9') {
            fileName[BASE_NAME_SIZE + 1]++;
        } else if (fileName[BASE_NAME_SIZE] != '9') {
            fileName[BASE_NAME_SIZE + 1] = '0';
            fileName[BASE_NAME_SIZE]++;
        } else {
            error("Can't create file name");
        }
    }
    if (!file.open(fileName, O_CREAT | O_WRITE | O_EXCL)) {
        error("file.open");
    }
    do {
        delay(10);
    } while (Serial.read() >= 0);
    
    Serial.print(F("Logging to: "));
    Serial.println(fileName);
    
    // Write data header.
    writeHeader();
    
    // Set BLE Shield name here, max. length 10
    ble_set_name("AWTAP00001");

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
    logIfAppropriate();
    
    handleBleCommands();
    
    // send out any outstanding data
    ble_do_events();
}