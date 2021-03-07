#include <Arduino.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include <MQTT.h>
#include <SAMD21Step.h> 
#include <MotionCtrl.h>



// Pin Definitions
const int S_PUL = 11;
const int SE_PUL = 7;
const int S_DIR = 9;
const int SE_DIR = A2;
const int SERVO_ENA = 10;
const int S_LIM = 3;
const int SE_LIM = A7;
const int S_ALM = 4;
const int SE_ALM = A1;

const int PWM1 = 6;
const int PWM2 = 5;

// Servo Settings:
#define STEP_PER_REV      3200   // How many steps per revolution of the motor (S1 on)
#define PULLEY_TEETH      15     // How many teeth has the pulley
#define BELT_PITCH        3      // What is the timing belt pitch in mm
#define MAX_TRAVEL        200    // What is the maximum physical travel in mm
#define KEEPOUT_BOUNDARY  5      // Soft endstop preventing hard crashes in mm. Will be 
                                 // subtracted twice from MAX_TRAVEL
#define MAX_RPM           2900   // What is the maximum RPM of the servo
#define MAX_ACC           30000  // Maximum acceleration in mm/s^2
#define INVERSE           false  // Set to true to invert the direction signal
                                 // The firmware expects the home switch to be located at the 
                                 // end of an retraction move. That way the machine homes 
                                 // itself away from the body.

// Derived Servo Settings
#define STEP_PER_MM       STEP_PER_REV / (PULLEY_TEETH * BELT_PITCH)
#define TRAVEL            MAX_TRAVEL - (2 * KEEPOUT_BOUNDARY)
#define MIN_STEP          int(0.5 + KEEPOUT_BOUNDARY * STEP_PER_MM)
#define MAX_STEP          int(0.5 + (MAX_TRAVEL - KEEPOUT_BOUNDARY) * STEP_PER_MM)
#define MAX_STEP_PER_SEC  int(0.5 + (MAX_RPM * STEP_PER_REV) / 60)
#define MAX_STEP_ACC      int(0.5 + MAX_ACC * STEP_PER_MM)

// Coniguration of the stepper motor
StepperConfig stepper0config = {
  .pulsePin = S_PUL,                   // Arduino pin number for the pulse pin. Must be a pin with a TCC timer output (see PinLookup). One timer can exclusivly serve only one pin.
  .dirPin = S_DIR,                     // Arduino pin number of the direction signal. May be any pin.
  .enablePin = SERVO_ENA,              // Arduino pin number of the driver enable signal. May be any pin. Set to -1 if not used.
  .enableActiveLow = true,             // Set to true if the enable signal is active low. For active high set to false.
  .alarmPin = S_ALM,                   // Arduino pin number of the driver alarm signal. Must be an interrupt pin. Set to -1 if not used.
  .alarmActiveHigh = true,             // Set to true if the alarm signal is active high. For active low set to false.
  .homePin = S_LIM,                    // Arduino pin number of the home switch. Must be an interrupt pin. Homing is mandatory
  .homeActiveHigh = true,              // Set to true if the home signal is active high. For active low set to false.
  .invertPulse = true,                 // Set to true if you need an active low pulse signal
  .minPulseWidth = 4.0,                // Minimum pulse width in µs
  .maxVelocity = MAX_STEP_PER_SEC,     // Maximum velocity in steps/sec
  .maxAcceleration = MAX_STEP_ACC,     // Maximum accelceration & decceleratin in steps/sec^2
  .minPosition = MIN_STEP,             // Minimum position of the axis in steps
  .maxPosition = MAX_STEP,             // Maximum position of the axis in steps
  .homePosition = 0,                   // The position of home. When the home switch is triggered position will be set to this value. May be outside the minPosition and maxPosition boundary
  .invertDirection = false,            // Set to true to invert the movement direction
  .invertHoming = false                // Set to true to invert the direction for searching the home switch
};

// timer tester
Stepper stepper0;
MotionCtrl& motion = MotionCtrl::instance();

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  
  Serial.println("Timer callback tester");

  stepper0.begin(&stepper0config);
  motion.begin(5.0, &stepper0);
  
  stepper0.enableDriver(true);
}

void loop() {

}





/*
WiFiClient net;
MQTTClient client(128); //The passed value denotes the read and write buffer size.

char ssid[] = "Kellerkind";
char pass[] = "Putzplan"; 
byte mac[6]; // the MAC address of your Wifi shield

char host[] = "192.168.1.24"; // Note: Local domain names (e.g. "Computer.local" on OSX) are not supported by Arduino. You need to set the IP address directly.
char user[] = "";
char pwd[] = "";
char clientID[] = "FuckIO"; //MQTT ClientID
char topic[] = "FuckIO/control/+";

unsigned int currentMillis = 0;
unsigned int startMillis = 0;
unsigned int microsPulses = 125;

void messageReceived(String &topic, String &payload) { 
bool bvalue = false;
  Serial.println("Message: " + topic + " - " + payload);
  if (topic.startsWith("FuckIO/control")) {
    topic.remove(0, topic.lastIndexOf('/') + 1);
    if (topic.equals("run")) {
      bvalue = payload.equalsIgnoreCase("TRUE");
      digitalWrite(SERVO_ENA, bvalue);  // Enable or disable the servo motors
      digitalWrite(LED_BUILTIN, bvalue);  // Enable or disable the servo motors
      // Handle Timer
    }
    if (topic.equals("rpm")) {
      microsPulses = int((1.0/((payload.toInt() / 60.0)*8000))*1000000);
      Serial.println(microsPulses);
    }
    if (topic.equals("dir")) {
      bvalue = payload.equalsIgnoreCase("TRUE");
      digitalWrite(S_DIR, bvalue);  // CW / CWW rotation
      digitalWrite(SE_DIR, bvalue);  // CW / CWW rotation
    }
    if (topic.equals("pwm")) {
      analogWrite(PWM1, constrain(payload.toInt(), 0, 255));
      analogWrite(PWM2, constrain(payload.toInt(), 0, 255));
    }
  }
}

void connect() {
  String macAdress = "";

  WiFi.begin(ssid, pass);

  Serial.print("connecting WiFi ");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  // print the SSID of the network you're attached to:
  Serial.print("\nSSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print your MAC address:
  WiFi.macAddress(mac);
  Serial.print("MAC Address: ");
  Serial.print(mac[5],HEX);
  Serial.print(":");
  Serial.print(mac[4],HEX);
  Serial.print(":");
  Serial.print(mac[3],HEX);
  Serial.print(":");
  Serial.print(mac[2],HEX);
  Serial.print(":");
  Serial.print(mac[1],HEX);
  Serial.print(":");
  Serial.println(mac[0],HEX);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI): ");
  Serial.print(rssi);
  Serial.println(" dBm");

  client.begin(host, net);

  Serial.print("connecting to MQTT-Server " + String(host));
  while (client.connect(clientID, user, pwd) == false) {
    Serial.print("*");
    delay(500);
  }

  Serial.println("\nsubscribing to " + String(topic));
  client.onMessage(messageReceived);
  if (client.subscribe(topic))
  {
    Serial.println("Success");
  } else
  {
    Serial.println("\nCouldn't subscribe");
  } 

  client.publish("/hello", "world");

}

void setup() {

  // Initialize Pins
  pinMode(S_PUL, OUTPUT);
  pinMode(SE_PUL, OUTPUT);
  pinMode(S_DIR, OUTPUT);
  pinMode(SE_DIR, OUTPUT);
  digitalWrite(S_DIR, HIGH);
  digitalWrite(SE_DIR, HIGH);
  pinMode(SERVO_ENA, OUTPUT);
  digitalWrite(SERVO_ENA, LOW);  //Disarm the servo motors
  pinMode(S_LIM, INPUT);
  pinMode(SE_LIM, INPUT);
  pinMode(S_ALM, INPUT);
  pinMode(SE_ALM, INPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.begin(115200);
  connect();
}

void loop() {
    
  currentMillis = millis();
  
  if (currentMillis - startMillis >= 100) {
    startMillis = currentMillis;
    client.loop();
    Serial.println("signal strength (RSSI): " + WiFi.RSSI());

    if (!client.connected()) {
      Serial.println("Lost connection! Try to reconnect.");
      connect();
    }
  }

  digitalWrite(S_PUL, HIGH);
  digitalWrite(SE_PUL, HIGH);
  delayMicroseconds(microsPulses);
  digitalWrite(S_PUL, LOW);
  digitalWrite(SE_PUL, LOW);
  delayMicroseconds(microsPulses);

}
*/