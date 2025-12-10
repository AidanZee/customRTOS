#define ENABLE_USER_AUTH
#define ENABLE_DATABASE
#define RXD1 7 // GPIO 7 for receiving data (RX)
#define TXD1 6 // GPIO 6 for transmitting data (TX)
#define BAUDRATE 115200

#include <Arduino.h>
#if defined(ESP32)
  #include <WiFi.h>
#elif defined(ESP8266)
  #include <ESP8266WiFi.h>
#endif
#include <WiFiClientSecure.h>
#include <FirebaseClient.h>



// Define the size of our buffer to hold the incoming message
const int MAX_MESSAGE_LENGTH = 64;
char receivedMessage[MAX_MESSAGE_LENGTH];
int messageIndex = 0; // Tracks the current position in the buffer


// Network and Firebase credentials
#define WIFI_SSID "TTUguest"
#define WIFI_PASSWORD "**************"
#define Web_API_KEY "***********************"
#define DATABASE_URL "https://rtos-database-b76f6-default-rtdb.firebaseio.com/"
#define USER_EMAIL "samkalu@ttu.edu"
#define USER_PASS "********"

// User functions
void processData(AsyncResult &aResult);

// Authentication
UserAuth user_auth(Web_API_KEY, USER_EMAIL, USER_PASS);

// Firebase components
FirebaseApp app;
WiFiClientSecure ssl_client;
using AsyncClient = AsyncClientClass;
AsyncClient aClient(ssl_client);
RealtimeDatabase Database;

// Timer variables for sending data every 10 seconds
unsigned long lastSendTime = 0;
const unsigned long sendInterval = 2000; // 10 seconds in milliseconds

// Variables to send to the database
String databasePath;
String Sample1Path;
String Sample2Path;
String sample1;
String sample2;

void setup(){
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, RXD1, TXD1);
  delay(1000);

  // Connect to Wi-Fi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED)    {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  // Configure SSL client
  ssl_client.setInsecure();
  #if defined(ESP32)
    ssl_client.setConnectionTimeout(1000);
    ssl_client.setHandshakeTimeout(5);
  #elif defined(ESP8266)
    ssl_client.setTimeout(1000); // Set connection timeout
    ssl_client.setBufferSizes(4096, 1024); // Set buffer sizes
  #endif

  // Initialize Firebase
  initializeApp(aClient, app, getAuth(user_auth), processData, "🔐 authTask");
  app.getApp<RealtimeDatabase>(Database);
  Database.url(DATABASE_URL);
}

void loop(){
  // Maintain authentication and async tasks
  app.loop();
  // Check if authentication is ready
  if (app.ready()){
    // Periodic data sending every 10 seconds
    unsigned long currentTime = millis();
    // Check if there is any data available to read on Serial1
    while (Serial1.available() > 0) {
      // Read the incoming byte
      char incomingChar = Serial1.read();
      if (incomingChar == '\n') {
        // Terminate the string (add the null character)
        receivedMessage[messageIndex] = '\0';
        if (String(receivedMessage).startsWith("Sample1:")) {
          sample1 = String(receivedMessage);
        }
        else if (String(receivedMessage).startsWith("Sample2:")) {
          sample2 = String(receivedMessage);
        }
        // Only process if we actually received content before the newline
        if (messageIndex > 0) {
          // Output the full message to the Serial Monitor (USB port)
          Serial.print("Received Full Line: ");
          Serial.println(receivedMessage);
        }
        // Reset the buffer index for the next message
        messageIndex = 0;
        // Exit the loop for processing the current line
        break;
      }
      // Ignore Carriage Returns: If we receive a carriage return ('\r').
      else if (incomingChar == '\r') {
        continue;
      }
      // Store the character if there is space in the buffer.
      else if (messageIndex < MAX_MESSAGE_LENGTH - 1) {
        receivedMessage[messageIndex] = incomingChar;
        messageIndex++;
      }
      else {
        // If the buffer is full before a newline, flush it and restart.
        Serial.println("WARNING: Message too long. Buffer flushed.");
        messageIndex = 0;
      }
    }

    if (currentTime - lastSendTime >= sendInterval){
      // Update the last send time
      lastSendTime = currentTime;
           
      // Get User UID
      Firebase.printf("User UID: %s\n", app.getUid().c_str());
      String uid = app.getUid().c_str();
      databasePath = "UsersData/" + uid;

      Sample1Path = databasePath + "/sample1";
      Sample2Path = databasePath + "/sample2";

      // send samples 1 & 2
      Database.set<String>(aClient, Sample1Path, sample1, processData, "RTDB_Send_String");
      Database.set<String>(aClient, Sample2Path, sample2, processData, "RTDB_Send_String");
    }  
  }
}

void processData(AsyncResult &aResult){
  if (!aResult.isResult())
    return;

  if (aResult.isEvent())
    Firebase.printf("Event task: %s, msg: %s, code: %d\n", aResult.uid().c_str(), aResult.eventLog().message().c_str(), aResult.eventLog().code());

  if (aResult.isDebug())
    Firebase.printf("Debug task: %s, msg: %s\n", aResult.uid().c_str(), aResult.debug().c_str());

  if (aResult.isError())
    Firebase.printf("Error task: %s, msg: %s, code: %d\n", aResult.uid().c_str(), aResult.error().message().c_str(), aResult.error().code());

  if (aResult.available())
    Firebase.printf("task: %s, payload: %s\n", aResult.uid().c_str(), aResult.c_str());
}
