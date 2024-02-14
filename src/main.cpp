#include <spi.h>
#include <Wire.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <Adafruit_PWMServoDriver.h>
#include <config.h> //enthält Zugangsdaten für das Heimnetz

#define PCA9685_ADDR 0x40 // Adresse des PCA9685-Treibers
#define SERVO_MIN_PULSEWIDTH 900 //650
#define SERVO_MAX_PULSEWIDTH 2100 //2350
//#define SERVO_MAX_DEGREE 180
#define SERVO_FREQ 50


#define DEVICE "armUnit" // hier den unit-name aendern
enum ServoID {
  TURM = 0,
  ARM = 1,
  ARM1 = 2,
  GREIFER = 3
};

// Pin-Nummern für die Servos auf dem PCA9685-Controller
const int servoPins[] = {0, 1, 2, 3}; // Beispielwerte, anpassen entsprechend der tatsächlichen Verdrahtung
const int servoDegrees[] = {180, 180, 180, 180};
//const int servoMinPWM[] = {90, 90, 180, 180};
//const int servoMaxPWM[] = {90, 90, 180, 180};


Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);

bool servoExecuted = false;

WiFiClient espClient;
PubSubClient client(espClient);

ServoID getServoID(const char* typeString) {
    Serial.print("Token: ");
    Serial.println(typeString);
    if (strcmp(typeString, "turm") == 0) {
        Serial.println("ServoId erkannt: turm");
        return TURM;
    } else if (strcmp(typeString, "arm") == 0) {
        return ARM;
    } else if (strcmp(typeString, "arm1") == 0) {
        return ARM1;
    } else if (strcmp(typeString, "greifer") == 0) {
        return GREIFER;
    } else {
        // Default-Wert oder Fehlerbehandlung
        return TURM; // Zum Beispiel, hier könnte auch ein Default-Wert zurückgegeben werden
    }
}


void controlServo(int servoIndex, int pos) {
  // Begrenze die Position auf den erwarteten Bereich (0-180)
  if(servoIndex >= 0 && servoIndex <= 1){
    pos = constrain(pos, 0, servoDegrees[servoIndex]);
  } else {
    pos = constrain(pos, 0, servoDegrees[servoIndex]);
  };
  
  // Konvertiere die Position von Grad in Pulsweite
  uint16_t pulse = map(pos, 0, servoDegrees[servoIndex], SERVO_MIN_PULSEWIDTH, SERVO_MAX_PULSEWIDTH);
  Serial.println(pulse);
  // Setze die Pulsweite für den entsprechenden Servo-Kanal auf dem PCA9685-Controller
  pwm.setPWM(servoPins[servoIndex], 0, pulse);
}

String formatStatusMessage(int servoID, int pos) {
  return "Servo-ID " + String(servoID) + " auf " + String(pos) + " Grad gefahren";
}

void parsePayload(int servoID, int pos) {
  // Steuern Sie den entsprechenden Servo auf dem PCA9685-Controller
  controlServo(servoID, pos);

  // Veröffentlichen Sie eine Bestätigungsnachricht
  String topic = String(DEVICE) + "/status";
  String statusMessage = "Servo-ID " + String(servoID) + " auf " + String(pos) + " Grad gefahren";
  client.publish(topic.c_str(), statusMessage.c_str());
}


void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.println("] ");

  // Extrahiere Servo-Typ und Servo-ID aus dem Topic
  char* token = strtok(topic, "/");
  // Ignoriere den ersten Token (Gerätenamen)
  token = strtok(NULL, "/");
  char* servoType = token;
  int servoID = getServoID(servoType);
  token = strtok(NULL, "/");
  Serial.print("servoID: ");
  Serial.println(servoID);

  char posString[10];
  memcpy(posString, payload, length);
  posString[length] = '\0'; // Nullterminierung für die Konvertierung
  int pos = atoi(posString);


  // Verarbeite die Nachricht nur, wenn die Servo-ID im gültigen Bereich liegt
  if (servoID >= TURM && servoID <= GREIFER) {
      // Veröffentliche die verarbeiteten Daten
      //char topicProcessed[50];
      //snprintf(topicProcessed, sizeof(topicProcessed), "%s/%d/processed", servoType, servoID);
      //client.publish(topicProcessed, servoType); // Veröffentliche das ursprüngliche Payload unter dem neuen Topic

      // Payload weiter verarbeiten
      parsePayload(servoID, pos);
  } else {
      // Ungültige Servo-ID
      client.publish((String(DEVICE) + "/status").c_str(), ("Ungültige Servo-ID: " + String(servoID)).c_str());
  }
}



void reconnect() {
  // Wiederherstellen der MQTT-Verbindung
  while (!client.connected()) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("No WIFI");
    }
    Serial.print("Attempting MQTT connection...");
    if (client.connect(DEVICE, mqttUser, mqttPassword)) {
      Serial.println("connected");
      // Hier kannst du die Nachricht senden
      client.publish(DEVICE"/status", "Connected");
      // Abonnieren von Nachrichten
      client.subscribe(DEVICE "/turm/pos");
      client.subscribe(DEVICE "/arm/pos");
      client.subscribe(DEVICE "/arm1/pos");
      client.subscribe(DEVICE "/greifer/pos");
;
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}


void setup() {
  // Beginnen Sie mit der seriellen Kommunikation
  Serial.begin(115200);
  Serial.println();
  Serial.println("Booting...");

  // WiFi-Modus auf Station (STA) setzen
  WiFi.mode(WIFI_STA);
  
  // Verbindung zum vorhandenen WLAN-Netzwerk herstellen
  WiFi.begin(ssid, password);
  Serial.print("Verbindung zum WLAN herstellen");
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
  }
  Serial.println("");
  Serial.println("Verbunden mit WLAN");

  // Überprüfen, ob eine Verbindung hergestellt wurde
  if (WiFi.status() != WL_CONNECTED) {
      Serial.println("Keine Verbindung zum WLAN hergestellt. Access Point starten.");

      // WiFi-Modus auf Access Point (AP) setzen
      WiFi.mode(WIFI_AP);
      
      // Access Point konfigurieren
      WiFi.softAP(hotspotSSID, hotspotPassword);
      Serial.println("Access Point gestartet.");
  }

  // Initialisiere ArduinoOTA
  ArduinoOTA.onStart([]() {
      Serial.println("Starte OTA-Aktualisierung");
  });

  ArduinoOTA.onEnd([]() {
      Serial.println("\nOTA-Aktualisierung abgeschlossen");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Fortschritt: %u%%\r", (progress / (total / 100)));
  });

  ArduinoOTA.onError([](ota_error_t error) {
      Serial.printf("OTA-Fehler [%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Authentifizierung fehlgeschlagen");
      else if (error == OTA_BEGIN_ERROR) Serial.println("OTA-Begin fehlgeschlagen");
      else if (error == OTA_CONNECT_ERROR) Serial.println("OTA-Verbindung fehlgeschlagen");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("OTA-Empfangsfehler");
      else if (error == OTA_END_ERROR) Serial.println("OTA-Endfehler");
  });

  ArduinoOTA.begin();
  
  // initialisiere MQTT
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);

  Wire.begin();

  // Initialisiere Servo PWM
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

}

void loop() 
{
  ArduinoOTA.handle();

  // Überprüfen, ob eine Verbindung zum MQTT-Broker hergestellt ist
  if (!client.connected()) {
    reconnect();
  }
  // MQTT-Nachrichten verarbeiten
  client.loop();

  // Senden Sie eine MQTT-Nachricht, wenn der Servobefehl ausgeführt wurde
  if (servoExecuted) {
    client.publish(DEVICE"/status", "Servo command executed");
    servoExecuted = false;
  }
}
