#include <Wire.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <Adafruit_PWMServoDriver.h>
#include <config.h> // Enthält Zugangsdaten für das Heimnetz

// Servo-Einstellungen
#define PCA9685_ADDR 0x40 //default 0x40
#define FREQ 50
#define SERVOSTEPTIME 5
#define DEVICE "armUnit"
#define MQTT_CONNECTION_TIMEOUT -4
#define MQTT_CONNECTION_LOST -3
#define MQTT_CONNECT_FAILED -2

const int minStep = 1;
const int servoPins[] = {0, 1, 2, 3};
const int servoDegrees[] = {180, 180, 180, 180}; // Servo Hardware max
const int SERVOMIN[] = {100, 100, 100, 100}; //{120, 150, 100, 170};
const int SERVOMAX[] = {590, 590, 590, 590}; //{530, 590, 450, 590};
const int servoMinPos[] = {0, 0, 0, 0}; // {0, 20, 0, 7};
const int servoMaxPos[] = {180, 180, 180, 180}; // {180, 90, 90, 80};
// schönste pos Turm: 175, Arm: 60, Arm1: 120, Greifer: 20

//init-Werte -> zur Laufzeit die aktuelle Position des Servos!
uint16_t currentServoPos[] = {60, 60, 60, 60}; 
uint8_t smoothness = 100; // Wartezeit zwischen den Schritten
bool servosInitialized = false;
bool rangeControl_ON = true;

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

/// @brief Ansteuerung des Servos
/// @param servoID 0 - 3
/// @param pulse in PWM
void rawServoMovement(int servoID, int pulse) {
  if (pulse >= SERVOMIN[servoID] && pulse <= SERVOMAX[servoID]) {
    Serial.print("raw Movement to: ");
    Serial.println(pulse);
    pwm.setPWM(servoPins[servoID], 0, pulse);
    currentServoPos[servoID] = map(pulse, SERVOMIN[servoID], SERVOMAX[servoID], 0, servoDegrees[servoID]);
    client.publish(String(DEVICE "/status/raw_in_degree").c_str(), ("Pulse: " + String(pulse) + " --> Grad: " + String(currentServoPos[servoID])).c_str());
  } else {
    client.publish(String(DEVICE "/status/raw_in_degree").c_str(), ("Invalid raw value: " + String(pulse)).c_str());
  }
}

//Macro Positionen
void moveToGripperPos(int stepDelay){
  if (stepDelay < 10000){
    rawServoMovement(TURM, 510);
    delay(stepDelay);
    rawServoMovement(GREIFER, 380);
    delay(stepDelay);
    rawServoMovement(ARM1, 480);
    rawServoMovement(ARM, 210);
    delay(stepDelay);
    rawServoMovement(GREIFER, 230);
    delay(stepDelay);
  }
}

void moveToTrashPos(int stepDelay){
  if (stepDelay < 10000){
    rawServoMovement(ARM, 400);
    rawServoMovement(ARM1, 300);
    delay(stepDelay);
    rawServoMovement(TURM, 120);
    delay(stepDelay);
    rawServoMovement(GREIFER, 380);
    delay(stepDelay);
    rawServoMovement(ARM1, 210);
    delay(stepDelay);
  }
}

/// @brief Ansteuerung des Servos 
/// @param servoID 0-3
/// @param pos in Grad
void controlServo(int servoID, int pos) {
  Serial.print("Movenment to pos: ");
  uint16_t pulse = map(pos, 0, servoDegrees[servoID], SERVOMIN[servoID], SERVOMAX[servoID]);
  Serial.print(pos);
  Serial.print(" ; Pulse: ");
  Serial.println(pulse);
  pwm.setPWM(servoPins[servoID], 0, pulse);
  currentServoPos[servoID] = pos;
}

/// @brief Ansteuerung des Servos 
/// @param servoID 0-3
/// @param pos in Grad
/// @param smoothness Zeit zwischen den Schritten
void controlServo(int servoID, int targetPos, int smoothness) {
  int step = (targetPos > currentServoPos[servoID]) ? 1 : -1; // Richtung 
  const int numSteps = abs(targetPos - currentServoPos[servoID]); // Anzahl der Schritte, die benötigt werden, um das Ziel zu erreichen
  const float acceleration = 2.0 / numSteps; // Beschleunigungsfaktor
  
  int currentStep = 0;
  int currentPos = currentServoPos[servoID];
  int stepDelay = smoothness;
  
  while (currentPos != targetPos) {
    // Berechnen die proportionale Position zwischen dem aktuellen und dem Ziel
    float proportion = float(currentStep) / numSteps;
    
    // Verwenden der Beschleunigungsfunktion, um die Bewegung zu steuern
    float factor = 0.5 - 0.5 * cos(proportion * PI);
    int newPos = currentPos + int(factor * (targetPos - currentPos));
    
    // Setzen der Position des Servos
    uint16_t pulse = map(newPos, 0, servoDegrees[servoID], SERVOMIN[servoID], SERVOMAX[servoID]);
    pwm.setPWM(servoPins[servoID], 0, pulse);
    
    // Berechnen der Zeit die zwischen den Schritten basierend auf der aktuellen Geschwindigkeit
    stepDelay = int(smoothness * (1.0 - factor));
    
    // berechnete Zeit warten, um die Geschwindigkeit zu steuern
    delay(stepDelay);
    
    // Aktualisieren der Position des Servos und des aktuellen Schritts
    currentPos = newPos;
    currentStep++;
  }
  
  // Setzen der Zielposition und aktualisieren der aktuellen Position
  uint16_t pulse = map(targetPos, 0, servoDegrees[servoID], SERVOMIN[servoID], SERVOMAX[servoID]);
  pwm.setPWM(servoPins[servoID], 0, pulse);
  currentServoPos[servoID] = targetPos;
}

// Handler-Funktion für das "pos" Topic
void handlePos(int servoID, String message) {
    // Code zur Verarbeitung der Position für den angegebenen Servo
    int pos = message.toInt();
    if (pos >= servoMinPos[servoID] && pos <= servoMaxPos[servoID]) {
        controlServo(servoID, pos, 10);
        client.publish(String(DEVICE "/status").c_str(), "Servo position set");
    } else {
        client.publish(String(DEVICE "/status").c_str(), "Invalid servo position");
    }
}

// Handler-Funktion für das "raw" Topic
void handleRawPos(int servoID, String message) {
    // Code zur Verarbeitung der Position für den angegebenen Servo
    // Achtung keine Grenzwerte!
    int pos = message.toInt();
    client.publish(String(DEVICE "/status").c_str(), String(pos).c_str());
    rawServoMovement(servoID, pos);
}

// Handler-Funktion für das "smoothness" Topic
void handleSmoothness(int servoID, String message) {
    // Code zur Verarbeitung der "smoothness" Nachricht hier einfügen
}

// Handler-Funktion für das "rangecontrol_ON" Topic
void handleRangeControl(int servoID, String message) {
    // Code zur Verarbeitung der "rangecontrol_ON" Nachricht hier einfügen
}

void showHelp(){
  client.publish(String(DEVICE "/help").c_str(), "topic: <unit>/<component>/<command> \nexample: armUnit/turm/pos payload: 150 \ncomponents: arm, arm1, turm, greifer, macro, help \ncommands: pos, raw");
}

/// @brief Behandlung der MQTT Nachrichten
/// @param topic 
/// @param payload 
/// @param length 
void callback(char* topic, byte* payload, unsigned int length) {
  // Teile des Topics auslesen
  char *device = strtok(topic, "/");
  char *component = strtok(NULL, "/");
  char *command = strtok(NULL, "");

  //string in Kleinbuchstaben umwandeln
  device = strlwr(device);
  component = strlwr(component);
  command = strlwr(command);

  // Extrahiere die Nachricht aus dem Payload
  String value((char*)payload, length);

  if (strcmp(component, "macro") == 0) {
    int intValue = value.toInt();
    if (strcmp(command, "grip") == 0){
        moveToGripperPos(intValue);
    }
    else if (strcmp(command, "trash") == 0){
        moveToTrashPos(intValue);
    }
  }
  else if (strcmp(component, "help") == 0) {
    showHelp();
  }
  else if (strcmp(component, "arm") == 0 || strcmp(component, "arm1") == 0 || strcmp(component, "turm") == 0 || strcmp(component, "greifer") == 0){
    // Suche den passenden Handler für das Topic und rufe ihn auf
    if (strcmp(command, "pos") == 0) {
        handlePos(getServoID(component), value);
    } else if (strcmp(command, "smoothpos") == 0) {
        controlServo(getServoID(component), value.toInt(), smoothness);
    } else if (strcmp(command, "raw") == 0) {
        handleRawPos(getServoID(component), value);
    } else if (strcmp(command, "smoothness") == 0) {
        smoothness = value.toInt();
    } else if (strcmp(command, "rangecontrol_ON") == 0) {
        handleRangeControl(getServoID(component), value);
    } else {
      // Kein passender Handler gefunden
      String err = "No handler found for topic: " + String(topic);
      client.publish(String(DEVICE "/status").c_str(), err.c_str());
    }
  }
}

/// @brief MQTT Verbindungsbehandlung
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
      showHelp();
      ///abonnieren der topics
      client.subscribe(DEVICE "/+/pos");
      client.subscribe(DEVICE "/+/smoothpos");
      client.subscribe(DEVICE "/+/smoothness");
      client.subscribe(DEVICE "/+/rangecontrol_ON");
      client.subscribe(DEVICE "/+/raw");
      client.subscribe(DEVICE "/macro/+");      
      client.subscribe(DEVICE "/help/+");
    } else {
      int errorCode = client.state();
      if (errorCode == MQTT_CONNECTION_TIMEOUT) {
        Serial.println("MQTT connection timeout");
      } else if (errorCode == MQTT_CONNECTION_LOST) {
        Serial.println("MQTT connection lost");
      } else if (errorCode == MQTT_CONNECT_FAILED) {
        Serial.println("MQTT connection failed");
      } else {
        Serial.print("MQTT connection failed with error code: ");
        Serial.println(errorCode);
      }
      Serial.println("Retrying MQTT connection in 5 seconds...");
      delay(5000);
    }
  }
}


/// @brief Initialisierung der Servos um eine definierte Position zu erhalten
void initServos() {
  moveToTrashPos(500); //Homepos
  servosInitialized = true;
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
  Wire.begin(21, 22);
  pwm.begin();
  pwm.setPWMFreq(FREQ);
  initServos();
}

void loop() {
  ArduinoOTA.handle();
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}
