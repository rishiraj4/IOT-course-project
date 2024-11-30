#include <WiFi.h>
#include <PubSubClient.h>

// Function Prototypes
void connectToWiFi();
void reconnect();
void manageSeat(int pirPin, int relayPin, const char* motionTopic, const char* acStatusTopic, unsigned long &lastMotionTime);
void publishInitialStatus();

// WiFi credentials
const char* ssid = "Wokwi-GUEST";           
const char* password = "";                  
const char* mqtt_server = "broker.emqx.io"; 
WiFiClient espClient;
PubSubClient client(espClient);

// PIR Motion Sensor Pins
const int pirPin1 = 25;  // Seat 1 PIR motion sensor
const int pirPin2 = 22;  // Seat 2 PIR motion sensor

// Relay Pins to control AC
const int relayPin1 = 14; // Seat 1 relay
const int relayPin2 = 17; // Seat 2 relay

// Time variables for motion detection
unsigned long lastMotionTime1 = 0;
unsigned long lastMotionTime2 = 0;
unsigned long motionDelay = 1000;  

// MQTT Topics
const char* motionTopic1 = "seat1/motion";  
const char* motionTopic2 = "seat2/motion";  
const char* acStatusTopic1 = "seat1/ac";    
const char* acStatusTopic2 = "seat2/ac";    
const char* seatOccupyCountTopic = "seat/occupancyCount"; 

void setup() {
  Serial.begin(115200);

  pinMode(pirPin1, INPUT);
  pinMode(pirPin2, INPUT);

  pinMode(relayPin1, OUTPUT);
  pinMode(relayPin2, OUTPUT);
  digitalWrite(relayPin1, LOW);  // Initially keep AC 1 off
  digitalWrite(relayPin2, LOW);  // Initially keep AC 2 off

  connectToWiFi();

  client.setServer(mqtt_server, 1883);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Seat 1 motion and AC control
  manageSeat(pirPin1, relayPin1, motionTopic1, acStatusTopic1, lastMotionTime1);

  // Seat 2 motion and AC control
  manageSeat(pirPin2, relayPin2, motionTopic2, acStatusTopic2, lastMotionTime2);

  // Publish seat occupancy count
  int occupiedSeats = (digitalRead(pirPin1) == HIGH) + (digitalRead(pirPin2) == HIGH);
  char occupyCount[3];
  sprintf(occupyCount, "%d", occupiedSeats);
  client.publish(seatOccupyCountTopic, occupyCount);
}

// Function Definitions
void manageSeat(int pirPin, int relayPin, const char* motionTopic, const char* acStatusTopic, unsigned long &lastMotionTime) {
  int motionDetected = digitalRead(pirPin);

  if (motionDetected == HIGH) {  
    if (digitalRead(relayPin) == LOW) {  
      Serial.println("Motion Detected! Turning on AC.");
      digitalWrite(relayPin, HIGH);  
      client.publish(acStatusTopic, "ON"); 
    }
    lastMotionTime = millis();    

    // Publish motion detected to MQTT
    client.publish(motionTopic, "YES");
  } else {  // No motion detected
    // If no motion is detected and enough time has passed, turn off the AC
    if (millis() - lastMotionTime > motionDelay) {
      if (digitalRead(relayPin) == HIGH) {  
        Serial.println("No motion detected. Turning off AC.");
        digitalWrite(relayPin, LOW);  
        client.publish(acStatusTopic, "OFF");  
      }
      // Publish no motion detected to MQTT
      client.publish(motionTopic, "NO");
    }
  }
}

void connectToWiFi() {
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Attempting to connect to WiFi...");
  }

  Serial.println("Connected to WiFi");
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32_Client")) {
      Serial.println("connected");

      publishInitialStatus();
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void publishInitialStatus() {
  client.publish(motionTopic1, digitalRead(pirPin1) == HIGH ? "YES" : "NO");
  client.publish(motionTopic2, digitalRead(pirPin2) == HIGH ? "YES" : "NO");
  client.publish(acStatusTopic1, digitalRead(relayPin1) == HIGH ? "ON" : "OFF");
  client.publish(acStatusTopic2, digitalRead(relayPin2) == HIGH ? "ON" : "OFF");

  int occupiedSeats = (digitalRead(pirPin1) == HIGH) + (digitalRead(pirPin2) == HIGH);
  char occupyCount[3];
  sprintf(occupyCount, "%d", occupiedSeats);
  client.publish(seatOccupyCountTopic, occupyCount);
}