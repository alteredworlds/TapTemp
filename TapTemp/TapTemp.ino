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
#include <SD.h>
#include <RBL_nRF8001.h>

#include <Wire.h>
#include <RTCLib.h>


// Prototypes
//==============================================================================

// see: http://forum.arduino.cc/index.php?topic=48882.0
union U {
    uint32_t unixTime;
    uint8_t   data[4];
};

// Define variables and constants

// SD chip select pin.  Be sure to disable any other SPI devices such as Enet.
const uint8_t chipSelect = SS;

// want at least 1s between analog reads
const uint32_t MIN_ANALOG_READ_INTERVAL_uS = 1000000;

// want at least 1 record per 60 mins to demonstrate 'alive'
const uint32_t MAX_LOG_INTERVAL_uS = 3600 * 1000000;

// only want to record meaningful changes in analog reading
const uint16_t MIN_ANALOG_DELTA = 1;

// Log file (SD utility library)
File file;

// Time in micros for next data record.
uint32_t logTime = 0;

// Time in micros when last sensor read occurred
uint32_t sensorReadTime = 0;

// current active file name
char fileName[13];

// RTC time when data last logged
uint16_t lastWriteYear = 2015;
uint8_t lastWriteMonth = 9;
uint8_t lastWriteDay = 5;

//uint16_t lastWriteYear = 0;
//uint8_t lastWriteMonth = 0;
//uint8_t lastWriteDay = 0;

// Real Time Clock
RTC_DS1307 RTC;


// we have only a single temperature probe right now...
const uint8_t ANALOG_COUNT = 1;

// last measured analog data
uint16_t recordedData[ANALOG_COUNT];

bool readLine(File &f, char* line, size_t maxLen) {
    for (size_t n = 0; n < maxLen; n++) {
        int c = f.read();
        if ( c < 0 && n == 0) return false;  // EOF
        if (c < 0 || c == '\n') {
            line[n] = 0;
            return true;
        }
        line[n] = c;
    }
    return false; // line too long
}

// for now just reads a SINGLE analog data record
bool readRecord(long* v1, uint16_t* v2) {
    char line[40], *ptr, *str;
    if (!readLine(file, line, sizeof(line))) {
        return false;  // EOF or too long
    }
    *v1 = strtol(line, &ptr, 10);
    if (ptr == line) return false;  // bad number if equal
    while (*ptr) {
        if (*ptr++ == ',') break;
    }
    *v2 = (uint16_t)strtol(ptr, &str, 10);
    return str != ptr;  // true if number found
}

void serialPrintTime(DateTime now) {
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
    Serial.print(now.unixtime());
    Serial.print(") ");
}

boolean openFileForRead(char* fileName) {
    boolean retVal = false;
    // we expect fileName to be intialized at this point
    Serial.print(F("Opening file: "));
    Serial.println(fileName);
    
    file = SD.open(fileName, O_READ);
    if (!file) {
        Serial.println(F("Failed to open file"));
    } else {
        do {
            delay(10);
        } while (Serial.read() >= 0);
        
        Serial.print(F("Opened file: "));
        Serial.println(fileName);
        retVal = true;
    }
    return retVal;
}

void deriveFileName(char* buf, int8_t bufSize, const DateTime& now) {
    // find the new file name
    snprintf(buf, bufSize, "%4d%02d%02d.csv", now.year(), now.month(), now.day());
}

void openFileForWrite(const DateTime& now) {
    // capture current date information
    lastWriteDay = now.day();
    lastWriteMonth = now.month();
    lastWriteYear = now.year();
    
    // name the current log file
    deriveFileName(fileName, sizeof(fileName), now);
    
    // we expect fileName to be intialized at this point
    Serial.print(F("Opening file: "));
    Serial.println(fileName);
    
    file = SD.open(fileName, O_CREAT | O_APPEND | O_WRITE);
    if (!file) {
        Serial.println(F("file.open"));
        while (1) {;}
    }
    do {
        delay(10);
    } while (Serial.read() >= 0);
    
    Serial.print(F("Logging to: "));
    Serial.println(fileName);
}

void closeFile() {
    if (file) {
        // we have an existing file
        // close it
        Serial.print(F("Closing log file: "));
        Serial.println(file.name());
        file.close();
    }
}

void rollLog(const DateTime& now) {
    // close any existing log file
    closeFile();
    
    // go for it (file will be named in openFileForWrite)
    openFileForWrite(now);
}

void bleWriteData(uint32_t rtcTime, uint16_t* data) {
    if (ble_connected()) {
        // we have an active bluetooth le connection, so inform client of change
        Serial.println(F("Writing data to active BLE connection..."));
        ble_write_bytes((byte *)&rtcTime, 4);
        for (int8_t i = 0; i < ANALOG_COUNT; i++) {
            ble_write_bytes((byte *)&data[i], 2);
        }
        
    }
}

// Log a data record if we need to
//    (big enough delta reading OR time interval)
boolean logIfAppropriate() {
    boolean writeData = false;
    if (file) {
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
                if (ANALOG_COUNT > 1) {
                    delay(5);
                }
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
                DateTime now = RTC.now();
                if (lastWriteDay && (now.day() != lastWriteDay)) {
                    rollLog(now);
                }
                
                // write the time derived from RTC
                uint32_t rtcTime = now.unixtime();
                file.print(rtcTime);
                
                // DEBUG output
                Serial.print(F("  Logging: "));
                serialPrintTime(now);
                
                // Write ADC data to CSV record.
                for (i = 0; i < ANALOG_COUNT; i++) {
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
                file.flush();
                
                // all OK: update last logged time
                logTime = currentTime;

                // write the data to any connected BLE devices
                bleWriteData(rtcTime, recordedData);
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
            case 'G': { // transmit all data for a specified date
                // read the date, which should be next param
                U date;
                for (int idx = 3; idx >= 0; idx--) {
                    date.data[idx] = ble_read();
                }
                
                Serial.print(F("Received unixdate: "));
                Serial.println(date.unixTime);
                
                // derive filename for that date
                DateTime dt(date.unixTime);
                
                // close our open log file
                closeFile();
                
                // grab filename for supplied date
                char requestedFileName[13];
                deriveFileName(requestedFileName, sizeof(requestedFileName), dt);
                
                // open file
                if (openFileForRead(requestedFileName)) {                    
                    // to hold timestamp, analog_data[#] when read from each line
                    long unixTimestamp;
                    uint16_t sample[ANALOG_COUNT];
                    
                    // iterate over every line
                    while (readRecord(&unixTimestamp, &sample[0])) {
                        // send valid record via existing ble mechanism
                        bleWriteData(unixTimestamp, sample);
                    }
                    
                    // close the file
                    closeFile();
                }
                
                // now re-open the log file
                openFileForWrite(RTC.now());
                break;
            }
                
            default:
                Serial.print(F("Command not recognised"));
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
    
    // make sure that the default chip select pin is set to
    // output, even if you don't use it:
    pinMode(SS, OUTPUT);
    
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
    if (!SD.begin(10, 11, 12, 13)) {
        Serial.println("Card failed, or not present");
        // don't do anything more:
        while (1) ;
    }
    
    // most recent file open, please...
    openFileForWrite(RTC.now());
    
    // Set BLE Shield name here, max. length 10
    ble_set_name("AW_TAP_01");

    // Init. and start BLE library.
    ble_begin();
    
    // set data to zero (necessary?)
    for (uint8_t i = 0; i < ANALOG_COUNT; i++) {
        recordedData[i] = 0;
    }
    
    // last logged time set 0 => initial time based write
    logTime = 0;
}

void loop() {
    // only perform one of handleBleCommands, logIfAppropriate on a given loop
    handleBleCommands() || logIfAppropriate();
    
    // in any case, need to process outstanding BLE events (e.g.: send data)
    ble_do_events();
}