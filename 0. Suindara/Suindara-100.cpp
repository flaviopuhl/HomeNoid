/*
 _______  __   __  ___   __    _  ______   _______  ______    _______ 
|       ||  | |  ||   | |  |  | ||      | |   _   ||    _ |  |   _   |
|  _____||  | |  ||   | |   |_| ||  _    ||  |_|  ||   | ||  |  |_|  |
| |_____ |  |_|  ||   | |       || | |   ||       ||   |_||_ |       |
|_____  ||       ||   | |  _    || |_|   ||       ||    __  ||       |
 _____| ||       ||   | | | |   ||       ||   _   ||   |  | ||   _   |
|_______||_______||___| |_|  |__||______| |__| |__||___|  |_||__| |__|
  
 Name:     Suindara
 Date:     DEC 2021
 Author:   Flavio L Puhl Jr <flavio_puhl@hotmail.com> 
 GIT:      
 About:    MQTT subscribe (eclipse broker) and Thinger.io publisher 
 
Update comments                                      
+-----------------------------------------------------+------------------+---------------+
|               Feature added                         |     Version      |      Date     |
+-----------------------------------------------------+------------------+---------------+
| Initial Release                                     |      1.0.0       |     DEC/21    |
|                                                     |                  |               |
|                                                     |                  |               |
+-----------------------------------------------------+------------------+---------------+


Library versions                                       
+-----------------------------------------+------------------+-------------------------- +
|       Library                           |     Version      |          Creator          |
+-----------------------------------------+------------------+-------------------------- +
| #include <ESP8266WiFi.h>                |      1.2.7       |     Arduino Community     |
| #include <PubSubClient.h>               |      2.8.0       |     Nick O'Leary          |
| #include <ArduinoJson.h>                |      6.18.5      |     Benoît Blanchon       |
| #include <NTPClient.h>                  |      3.1.0       |     Arduino Community     |
| #include <WiFiUdp.h>                    |                  |                           |
| #include <ThingerESP8266.h>             |      2.21.1      |       thinger-io          |
+-----------------------------------------+------------------+-------------------------- +


Upload settings 
+----------------------------------------------------------------------------------------+
| PLATFORM: Espressif 8266 (3.2.0) > NodeMCU 1.0 (ESP-12E Module)                        |
| HARDWARE: ESP8266 160MHz, 80KB RAM, 4MB Flash                                          |
| PACKAGES:                                                                              |
|  - framework-arduinoespressif8266 3.30002.0 (3.0.2)                                    |
|  - tool-esptool 1.413.0 (4.13)                                                         |
|  - tool-esptoolpy 1.30000.201119 (3.0.0)                                               |
|  - toolchain-xtensa 2.100300.210717 (10.3.0)                                           |
|                                                                                        |
| RAM:   [====      ]  39.6% (used 32468 bytes from 81920 bytes)                         |
| Flash: [===       ]  30.0% (used 313621 bytes from 1044464 bytes)                      |
+----------------------------------------------------------------------------------------+

*/

/*+--------------------------------------------------------------------------------------+
 *| Libraries                                                                            |
 *+--------------------------------------------------------------------------------------+ */
// Libraries built into IDE
#include <Arduino.h>
#ifdef ESP8266
  #include <ESP8266WiFi.h>
#else
  #include <WiFi.h>
#endif
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#define _DISABLE_TLS_                                 // Needed for Thinger
#include <ThingerESP8266.h>                           // Thinger

/*+--------------------------------------------------------------------------------------+
 *| Constants declaration                                                                |
 *+--------------------------------------------------------------------------------------+ */
 
const char *ssid =  "CasaDoTheodoro";                 // name of your WiFi network
const char *password =  "09012011";                   // password of the WiFi network

const char *ID = "SuinaraDev";                        // Name of our device, must be unique
const char *TOPIC1 = "Seriema/data";                  // Topic to subcribe to

const char* BROKER_MQTT = "mqtt.eclipseprojects.io";  // MQTT Cloud Broker URL

String swversion = __FILE__;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

WiFiClient wclient;
PubSubClient client(wclient);                         // Setup MQTT client

#define user "fpuhl"                                          /* Thinger *
#define device_Id "HomenoideGateway"                          // Thinger
#define device_credentials "jt0J73!BihBvbXe7"                 // Thinger
ThingerESP8266 thing(user, device_Id, device_credentials);    // Thinger

/*+--------------------------------------------------------------------------------------+
 *| Global Variables                                                                     |
 *+--------------------------------------------------------------------------------------+ */

unsigned long loop1 = 0;                             // stores the value of millis() in each iteration of loop()
unsigned long loop2 = 0; 

float uptime = 0;

char const* Device_topic0     = "empty";
String Version_topic0         = "empty";
float RSSI_topic0             = 0;
String IP_topic0              = "empty";
String LastRoll_topic0        = "empty";
int Uptime_topic0             = 0;

char const* Device_topic1     = "empty";
char const* Version_topic1    = "empty";
float RSSI_topic1             = 0;
char const* IP_topic1         = "empty";
char const* LastRoll_topic1   = "empty";
int Uptime_topic1             = 0;
float Temp_topic1             = 0;

/*+--------------------------------------------------------------------------------------+
 *| MQTT callback                                                                        |
 *+--------------------------------------------------------------------------------------+ */

// Handle incomming messages from the broker
void callback(char* topic, byte* payload, unsigned int length) {

  char str[length+1];
    Serial.print("Message arrived [");
      Serial.print(topic);
        Serial.print("] ");

  unsigned int i=0;
  for (i=0;i<length;i++) {
    //Serial.print((char)payload[i]);                               // print raw json data, debug only
    str[i]=(char)payload[i];
  }
  
  Serial.println();       
  str[i] = 0; // Null termination

 // JSON Deserialization
  StaticJsonDocument <256> doc;
  
  DeserializationError error = deserializeJson(doc, payload);
  
    // Test if parsing succeeds.
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      //Serial.println(error.f_str()); //https://github.com/bblanchon/ArduinoJson/issues/1525
      return;
    }

  

  if (strcmp(topic,TOPIC1)==0) {

    Device_topic1    = doc["Device"];
    Version_topic1   = doc["Version"];
    RSSI_topic1      = doc["RSSI (db)"];
    IP_topic1        = doc["IP"];
    LastRoll_topic1  = doc["LastRoll"];
    Uptime_topic1    = doc["Uptime (h)"];
    Temp_topic1      = doc["Temp (°C)"];

      Serial.print(" Device = ");     Serial.println(Device_topic1);
      Serial.print(" Version = ");    Serial.println(Version_topic1);
      Serial.print(" RSSI = ");       Serial.println(RSSI_topic1);
      Serial.print(" IP = ");         Serial.println(IP_topic1);
      Serial.print(" LastRoll = ");   Serial.println(LastRoll_topic1);
      Serial.print(" Uptime = ");     Serial.println(Uptime_topic1);
      Serial.print(" Temp = ");       Serial.println(Temp_topic1);
      Serial.print("\n \n");

    }

}

/*+--------------------------------------------------------------------------------------+
 *| Connect to WiFi network                                                              |
 *+--------------------------------------------------------------------------------------+ */

void setup_wifi() {
  Serial.print("\nConnecting to ");
  Serial.println(ssid);
    WiFi.mode(WIFI_STA);                              // Setup ESP in client mode
    WiFi.setSleepMode(WIFI_NONE_SLEEP);
    WiFi.begin(ssid, password);                       // Connect to network

    int wait_passes = 0;
    while (WiFi.status() != WL_CONNECTED) {           // Wait for connection
      delay(500);
      Serial.print(".");
      if (++wait_passes >= 20) { ESP.restart(); }     // Restart in case of no wifi connection   
    }

  Serial.println();
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  
}

/*+--------------------------------------------------------------------------------------+
 *| Reconnect to MQTT client                                                             |
 *+--------------------------------------------------------------------------------------+ */
 
void reconnect() {
  
  while (!client.connected()) {                             // Loop until we're reconnected
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(ID)) {
      Serial.println("connected");
      
      client.subscribe(TOPIC1);                             // Subscribe to MQTT
        Serial.print("Subscribing to: "); Serial.println(TOPIC1);
        Serial.print("All topics subscribed.");
        Serial.print("\n\n");

    } else {
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
      setup_wifi();
    }
  }
}

/*+--------------------------------------------------------------------------------------+
 *| Get Date & Time                                                                      |
 *+--------------------------------------------------------------------------------------+ */
 
String DateAndTime(){

    timeClient.setTimeOffset(-10800);                       // Set offset time in seconds to adjust for your timezone, for example:
                                                            // GMT +1 = 3600
                                                            // GMT +8 = 28800
                                                            // GMT -1 = -3600
                                                            // GMT 0 = 0
    while(!timeClient.update()) {
      timeClient.forceUpdate();
    }

  time_t epochTime = timeClient.getEpochTime();              // The time_t type is just an integer. 
                                                             // It is the number of seconds since the Epoch.
  struct tm * tm = localtime(&epochTime);
  char dts[22];
    strftime(dts, sizeof(dts), "%d%b%Y %H-%M-%S", tm);       // https://www.cplusplus.com/reference/ctime/strftime/
  
  return dts;
 
}

/*+--------------------------------------------------------------------------------------+
 *| Setup                                                                                |
 *+--------------------------------------------------------------------------------------+ */
 
void setup() {
  Serial.begin(115200);                               // Start serial communication at 115200 baud
    delay(100);
  
  swversion = (swversion.substring((swversion.indexOf(".")), (swversion.lastIndexOf("\\")) + 1));   
   Serial.print("SW version: ");
   Serial.println(swversion);
     
  setup_wifi();                                       // Connect to network

  Serial.println("Broker MQTT setting server.. ");	
    client.setServer(BROKER_MQTT, 1883);              // MQTT port, unsecure

  Serial.println("Starting timeclient server.. "); 	
    timeClient.begin();                                 /* Initialize a NTPClient to get time */  

  Serial.println("Initialize MQTT callback routine.. "); 	
    client.setCallback(callback);                     // Initialize the callback routine

}

/*+--------------------------------------------------------------------------------------+
 *| main loop                                                                            |
 *+--------------------------------------------------------------------------------------+ */
 
void loop() {

  unsigned long currentMillis = millis();             /* capture the latest value of millis() */
  uptime = millis()/3600000;                          /* Update uptime */

  if (!client.connected())                            // Reconnect if connection to MQTT is lost
  {    reconnect();      }

  
   if (currentMillis - loop2 >= 60*1000) {            /* Gateway device health */  
    Serial.println("Loop Thinger: Start");

      Device_topic0     = "Suindara";
      Version_topic0    = swversion;
      RSSI_topic0       = WiFi.RSSI();
      IP_topic0         = WiFi.localIP().toString();
      LastRoll_topic0   = DateAndTime();
      Uptime_topic0     = uptime;
     
        Serial.print(" Device = ");     Serial.println(Device_topic0);
        Serial.print(" Version = ");    Serial.println(Version_topic0);
        Serial.print(" RSSI = ");       Serial.println(RSSI_topic0);
        Serial.print(" IP = ");         Serial.println(IP_topic0);
        Serial.print(" LastRoll = ");   Serial.println(LastRoll_topic0);
        Serial.print(" Uptime = ");     Serial.println(Uptime_topic0);
        Serial.print("\n \n");

    Serial.println("Loop Thinger: End");
    loop2 = currentMillis;
  }      
  
  
  thing["data"] >> [](pson& out){  
    // Add the values and the corresponding code
    out["Device_topic0"]    = Device_topic0;
    out["Version_topic0"]   = Version_topic0;
    out["RSSI_topic0"]      = RSSI_topic0;
    out["IP_topic0"]        = IP_topic0;
    out["LastRoll_topic0"]  = LastRoll_topic0;
    out["Uptime_topic0"]    = Uptime_topic0;

    out["Device_topic1"]    = Device_topic1;
    out["Version_topic1"]   = Version_topic1;
    out["RSSI_topic1"]      = RSSI_topic1;
    out["IP_topic1"]        = IP_topic1;
    out["LastRoll_topic1"]  = LastRoll_topic1;
    out["Uptime_topic1"]    = Uptime_topic1;
    out["Temp"]             = Temp_topic1;
    }; 




  client.loop();                                   // MQTT
  thing.handle();                                 // Thinger
  
}

