#include "Particle.h"
#include "google-maps-device-locator.h"

void locationCallback(float lat, float lon, float accuracy);

SerialLogHandler logHandler;
GoogleMapsDeviceLocator locator;

SYSTEM_THREAD(ENABLED);


void setup() {
        Serial.begin(9600);
//        locator.withLocatePeriodic(60).withEventName("geoLocate").withSubscribe(locationCallback);
		locator.withEventName("geoLocate").withSubscribe(locationCallback);
}


void loop() {
        locator.loop();

        delay(3000);
      locator.publishLocation();
delay(1000);
}

void locationCallback(float lat, float lon, float accuracy) {
        Serial.printlnf("lat=%f lon=%f accuracy=%f", lat, lon, accuracy);
}
