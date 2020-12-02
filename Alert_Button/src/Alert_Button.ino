/*
 * Project Alert_Button
 * Description: Adding an alert button that will notify and start a trigger
 * Author: Janel Sanchez
 * Date: 01-Dec-2020
 */

#include <Adafruit_MQTT.h>

#include "Adafruit_MQTT/Adafruit_MQTT.h" 
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h" 
#include "Adafruit_MQTT/Adafruit_MQTT.h" 

#include "credentials.h"

TCPClient TheClient; 

Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY); 

Adafruit_MQTT_Publish button = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/alertButton");

unsigned long last;
const int buttonPin = D2;


void setup() {
  Serial.begin(9600);
  delay(100); //wait for Serial Monitor to startup

  pinMode (buttonPin, INPUT);
}

void loop() {
  MQTT_connect();
  MQTT_ping();
  pushAlertButton ();
}

void MQTT_connect() {
  int8_t ret;
 
  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }
 
  Serial.print("Connecting to MQTT... ");
 
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
  }
  Serial.println("MQTT Connected!");
}

void MQTT_ping() {
  if ((millis()-last)>120000) {
    Serial.printf("Pinging MQTT \n");
    if(! mqtt.ping()) {
      Serial.printf("Disconnecting \n");
      mqtt.disconnect();
    }
    last = millis();
  }
}

void pushAlertButton () {
  bool buttonState;
  static bool lastButton;

  buttonState = digitalRead(buttonPin);
  if(buttonState != lastButton) {
    if(buttonState == HIGH) {
      if(mqtt.Update()) {
        button.publish(buttonState);
        Serial.printf("Alert Button Pressed \n"); 
      } 
    }
  lastButton = buttonState;
  }
}