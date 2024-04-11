
#include <DovesLapTimer.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

// comment out when not using serial usb:
#define HAS_DEBUG
#define DEBUG_SERIAL Serial

#ifdef HAS_DEBUG
#define debugln DEBUG_SERIAL.println
#define debug DEBUG_SERIAL.print
#define debugWrite DEBUG_SERIAL.write
#else
void dummy_debug(...) {}
#define debug dummy_debug
#define debugln dummy_debug
#define debugWrite dummy_debug
#endif

// comment out if not using software serial with RX, TX on pins 9, 8
#define USING_SOFTWARE_SERIAL

#ifdef USING_SOFTWARE_SERIAL
SoftwareSerial gps_serial(9, 8);
#else
Serial1 gps_serial;
#endif
Adafruit_GPS GPS(&gps_serial);

// change these values to define start/stop line
const double crossingPointALat = 0.00;
const double crossingPointALng = 0.00;
const double crossingPointBLat = 0.00;
const double crossingPointBLng = 0.00;

// change this to set how far away from line to change lap number
double crossingThreshold = 7.0;
DovesLapTimer lapTimer(crossingThreshold);

unsigned long currentMillis;
unsigned long startTime;
unsigned long endTime;
unsigned long loopCounter;
const unsigned long updateInterval = 1000;
float frameRate = 0.0;

unsigned long getGpsTimeInMilliseconds() {
    unsigned long timeInMillis = 0;
    timeInMillis += GPS.hour * 3600000ULL; // Convert hours to milliseconds
    timeInMillis += GPS.minute * 60000ULL; // Convert minutes to milliseconds
    timeInMillis += GPS.seconds * 1000ULL; // Convert seconds to milliseconds
    timeInMillis += GPS.milliseconds;      // Add the milliseconds part

    return timeInMillis;
}

void gpsLoop() {
    char c = GPS.read();

#ifdef HAS_DEBUG
    debugWrite(c);
#endif

    if (GPS.newNMEAreceived() && GPS.parse(GPS.lastNMEA()))
    {
        if (GPS.fix)
        {
            lapTimer.updateCurrentTime(getGpsTimeInMilliseconds());
            float altitude = GPS.altitude;
            float speed = GPS.speed;
            lapTimer.loop(GPS.latitudeDegrees, GPS.longitudeDegrees, altitude, speed);
        }
    }
}

int getCurrentLap() {
    return lapTimer.getLaps();
}

void setup() {

    #ifdef HAS_DEBUG
        Serial.begin(115200);
        delay(5000);
        while (!Serial)
            ;
        debugln(F("Beginning debug mode of GPS module."));
    #endif

    GPS.begin(9600);

    // We only need RMC data (coords and such) so send this command:
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);

    // Set the update rate to 5 Hz
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
    delay(1000);

    lapTimer.setStartFinishLine(crossingPointALat, crossingPointALng, crossingPointBLat, crossingPointBLng);
    lapTimer.reset();
    lapTimer.forceLinearInterpolation();

    debugln(F("Started GPS lap timer"));

    // startTime = millis();
    // loopCounter = 0;
}

void loop() {
    // loopCounter++;
    // endTime = millis();

    // // Check if the update interval has passed
    // if (endTime - startTime >= updateInterval)
    // {
    //     // Calculate the frame rate (loops per second)
    //     frameRate = (float)loopCounter / ((endTime - startTime) / 1000.0);
    //     // Reset the loop counter and start time for the next interval
    //     loopCounter = 0;
    //     startTime = millis();
    // }

    // currentMillis = millis();
    gpsLoop();
}