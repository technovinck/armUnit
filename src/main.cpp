#include <Wire.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <Adafruit_PWMServoDriver.h>
#include <config.h> // Enthält Zugangsdaten für das Heimnetz

// Servo-Einstellungen
#define PCA9685_ADDR 0x40
//#define SERVOMIN  150
//#define SERVOMAX  590
#define SMOOTHNESS 2
#define INVALID_SERVO -1 // Beispielwert für eine ungültige Servo-ID
#define DEVICE "armUnit"

const int minStep = 1;
const int servoPins[] = {0, 1, 2, 3};
const int SERVOMIN[] = {150, 150, 100, 150};
const int SERVOMAX[] = {590, 590, 450, 590};
const int servoDegrees[] = {180, 180, 180, 180};
const int servoMinPos[] = {180, 180, 180, 10};
const int servoMaxPos[] = {180, 180, 180, 80};

bool servosInitialized = false;
uint16_t currentServoPos[] = {0, 0, 10, 80}; //auch init-Werte

enum ServoID {
  TURM = 0,
  ARM,
  ARM1,
  GREIFER,
  NUM_SERVOS
};

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);
WiFiClient espClient;
PubSubClient client(espClient);

ServoID getServoID(const char* typeString) {
    if (strcmp(typeString, "turm") == 0) return TURM;
    else if (strcmp(typeString, "arm") == 0) return ARM;
    else if (strcmp(typeString, "arm1") == 0) return ARM1;
    else if (strcmp(typeString, "greifer") == 0) return GREIFER;
    else return TURM;
}

#define KP 1.0 // Proportional gain
#define KI 0.0 // Integral gain
#define KD 0.0 // Derivative gain

double errorSum = 0.0; // Accumulated error for integral control
double lastError = 0.0; // Last error for derivative control
unsigned long lastTime = 0; // Last time for derivative control

void moveServosPID(int servoID, int targetPos) {
  // Get the current position
  int currentPos = currentServoPos[servoID];

  // Map the target position from 0-180 to the range 150-590
  uint16_t mappedTargetPos = map(targetPos, 0, 180, 150, 590);
  
  // Calculate the error
  double error = mappedTargetPos - currentPos;

  // Get current time
  unsigned long now = millis();
  // Calculate time difference
  double dt = (now - lastTime) / 1000.0; // Convert to seconds
  // Update last time
  lastTime = now;

  // Proportional control
  double P = KP * error;

  // Integral control
  errorSum += error * dt;
  double I = KI * errorSum;

  // Derivative control
  double D = KD * (error - lastError) / dt;
  lastError = error;

  // Calculate control signal
  double controlSignal = P + I + D;

  // Apply control signal to the servo
  int nextPos = constrain(currentPos + controlSignal, SERVOMIN[servoID], SERVOMAX[servoID]);
  pwm.setPWM(servoID, 0, nextPos);
  currentServoPos[servoID] = nextPos;
}


void rawServoMovement(int servoID, int pulse) {
  Serial.print("raw Movenment to: ");
  Serial.println(pulse);
  pwm.setPWM(servoPins[servoID], 0, pulse);
}


void controlServo(int servoID, int pos) {
  Serial.print("pos Movenment");
  uint16_t pulse = map(pos, 0, servoDegrees[servoID], SERVOMIN[servoID], SERVOMAX[servoID]);
  pwm.setPWM(servoPins[servoID], 0, pulse);
}

void parsePayload(int servoID, int pos) {
  if (pos <= servoMaxPos[servoID]) {
    controlServo(servoID, pos);
    client.publish(String(DEVICE "/status").c_str(), "Servo position set");
  } else {
    client.publish(String(DEVICE "/status").c_str(), "Invalid servo position");
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  char* token = strtok(topic, "/");
  token = strtok(NULL, "/");
  int servoID = getServoID(token);
  token = strtok(NULL, "/");
  
  if (token != NULL) {
    String subTopic = String(token);
    if (subTopic.equals("pos") || subTopic.equals("raw")) {
      int pos = atoi((char *)payload);
      if (servoID >= 0 && servoID < NUM_SERVOS) {
        if (subTopic.equals("raw")) rawServoMovement(servoID, pos);
        else parsePayload(servoID, pos);
      } else {
        client.publish(String(DEVICE "/status").c_str(), "Invalid servo ID");
      }
    }
  }
}


void reconnect() {
  while (!client.connected()) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("No WIFI");
      delay(500);
      return;
    }
    Serial.print("Attempting MQTT connection...");
    if (client.connect(DEVICE, mqttUser, mqttPassword)) {
      Serial.println("connected");
      client.publish(DEVICE "/status", "Connected");
      client.subscribe(DEVICE "/+/pos");
      client.subscribe(DEVICE "/+/raw");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void initServos() {
  for (int sID = 0; sID < NUM_SERVOS; sID++){
    controlServo(sID, currentServoPos[sID]);
  }
  Serial.println("ServoInit complete");
}

void setup() {
  Serial.begin(115200);
  Serial.println("Booting...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");
  ArduinoOTA.begin();
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  initServos();
}

void loop() {
  ArduinoOTA.handle();
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}
