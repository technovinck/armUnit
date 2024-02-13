#include <spi.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <Adafruit_PWMServoDriver.h>
#include <config.h> //enthält Zugangsdaten für das Heimnetz

#define PCA9685_ADDR 0x40 // Adresse des PCA9685-Treibers
#define SERVO_MIN_PULSEWIDTH 650
#define SERVO_MAX_PULSEWIDTH 2350
#define SERVO_MAX_DEGREE 180

#define DEVICE "armUnit" // hier den unit-name aendern

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);

bool servoExecuted = false;

WiFiClient espClient;
PubSubClient client(espClient);

// Funktion zum Empfangen von MQTT-Nachrichten
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  
  // Null-terminiertes Zeichenarray erstellen
  char message[length + 1]; // +1 für das Nullzeichen
  memcpy(message, payload, length);
  message[length] = '\0'; // Nullterminierung hinzufügen
  
  // Nachricht als String ausgeben
  client.publish(DEVICE"/status", "Nachricht erhalten!");

  // Hier können Sie die Payload weiter verarbeiten
}


void reconnect() {
  // Wiederherstellen der MQTT-Verbindung
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect(DEVICE, mqttUser, mqttPassword)) {
      Serial.println("connected");
      // Hier kannst du die Nachricht senden
      client.publish(DEVICE"/status", "Connected");
      // Abonnieren von Nachrichten
      client.subscribe(DEVICE"/control");
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

  // Initialisiere Servo PWM
  pwm.begin();
  pwm.setPWMFreq(60);  // Setze PWM-Frequenz auf 60 Hz

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
