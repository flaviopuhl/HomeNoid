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
| RAM:   [====      ]  40.6% (used 33224 bytes from 81920 bytes)                         |
| Flash: [===       ]  30.1% (used 314361 bytes from 1044464 bytes)                      |
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
#define THINGER_SERIAL_DEBUG

/*+--------------------------------------------------------------------------------------+
 *| Constants declaration                                                                |
 *+--------------------------------------------------------------------------------------+ */
 
const char *ssid =  "CasaDoTheodoro1";                         // name of your WiFi network
const char *password =  "09012011";                           // password of the WiFi network

const char *ID = "SuindaraDev";                               // Name of our device, must be unique
const char *TOPIC1 = "Seriema/data";                          // Topic to subcribe to
const char *TOPIC2 = "Harpia/data";                           // Topic to subcribe to

const char* BROKER_MQTT = "broker.hivemq.com";
//const char* BROKER_MQTT = "mqtt.eclipseprojects.io";          // MQTT Cloud Broker URL


String swversion = __FILE__;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

WiFiClient wclient;
PubSubClient client(wclient);                                 /* Setup MQTT client */

#define user "fpuhl"                                          /* Thinger */
#define device_Id "HomenoideGateway"                          /* Thinger */
#define device_credentials "jt0J73!BihBvbXe7"                 /* Thinger */
ThingerESP8266 thing(user, device_Id, device_credentials);    /* Thinger */

/*+--------------------------------------------------------------------------------------+
 *| Global Variables                                                                     |
 *+--------------------------------------------------------------------------------------+ */

unsigned long loop1 = 0;                             /* stores the value of millis() in each iteration of loop() */
unsigned long loop2 = 0; 

float uptime = 0;

char const* Device_topic0;
String      Version_topic0    = "empty";
float       RSSI_topic0       = 0;
String      IP_topic0         = "empty";
String      LastRoll_topic0   = "empty";
int         Uptime_topic0     = 0;

char const* Device_topic1     = "empty"; 
char const* Version_topic1    = "empty";
float       RSSI_topic1       = 0;
char const* IP_topic1         = "empty";
char const* LastRoll_topic1   = "empty";
int         Uptime_topic1     = -1;
float       Temp_topic1       = 0;

char const* Device_topic2     = "empty";
char const* Version_topic2    = "empty";
float       RSSI_topic2       = 0;
char const* IP_topic2         = "empty";
char const* LastRoll_topic2   = "empty";
int         Uptime_topic2     = -1;
char const* LastPicture_topic2= "empty";
float       Temp_topic2       = 0;

String Device_topic1str       = "empty";
String Device_topic2str       = "empty";
String Version_topic1str      = "empty";
String Version_topic2str      = "empty";
String IP_topic1str           = "empty";
String IP_topic2str           = "empty";
String LastRoll_topic1str     = "empty";
String LastRoll_topic2str     = "empty";

/*+--------------------------------------------------------------------------------------+
 *| MQTT callback                                                                        |
 *+--------------------------------------------------------------------------------------+ */

// Handle incomming messages from the broker
void callback(char* topic, byte* payload, unsigned int length) {

  char str[length+1] = ".";
    Serial.print("Message arrived [");
      Serial.print(topic);
        Serial.println("] ");

  unsigned int i=0;
  for (i=0;i<length;i++) {
    str[i]=(char)payload[i];
    //Serial.print((char)payload[i]);                               // print raw json data, debug only
    //Serial.print(str[i]);                                           // print raw json data, debug only
  }
  
  Serial.println();       
  str[i] = 0; // Null termination

 // JSON Deserialization
  StaticJsonDocument <1000> doc;
  
  DeserializationError error = deserializeJson(doc, payload);
  
    // Test if parsing succeeds.
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str()); //https://github.com/bblanchon/ArduinoJson/issues/1525
      return;
    } else {
      Serial.print(F("deserializeJson() success.. "));
    }

  if (strcmp(topic,TOPIC1)==0) {

    Serial.print("allocating data for [");
      Serial.print(topic);
      Serial.println("] ");
      Serial.println();

    Device_topic1    = doc["Device"];
      Device_topic1str = Device_topic1;
    Version_topic1   = doc["Version"];
      Version_topic1str = Version_topic1;
    RSSI_topic1      = doc["RSSI (db)"];
    IP_topic1        = doc["IP"];
      IP_topic1str = IP_topic1;
    LastRoll_topic1  = doc["LastRoll"];
      LastRoll_topic1str =  LastRoll_topic1;
    Uptime_topic1    = doc["UpTime (h)"];
    Temp_topic1      = doc["Temp (°C)"];

    Serial.print(" Topic1 Device    = ");    Serial.println(Device_topic1str);
    Serial.print(" Topic1 Version   = ");    Serial.println(Version_topic1str);
    Serial.print(" Topic1 RSSI      = ");    Serial.println(RSSI_topic1);
    Serial.print(" Topic1 IP        = ");    Serial.println(IP_topic1str);
    Serial.print(" Topic1 LastRoll  = ");    Serial.println(LastRoll_topic1str);
    Serial.print(" Topic1 Uptime    = ");    Serial.println(Uptime_topic1);
    Serial.print(" Topic1 Temp      = ");    Serial.println(Temp_topic1);
    Serial.println();

  }

  if (strcmp(topic,TOPIC2)==0) {

    Serial.print("allocating data for [");
      Serial.print(topic);
      Serial.println("] ");
      Serial.println();

    Device_topic2    = doc["Device"];
      Device_topic2str  = Device_topic2;
    Version_topic2   = doc["Version"];
      Version_topic2str = Version_topic2;
    RSSI_topic2      = doc["RSSI (db)"];
    IP_topic2        = doc["IP"];
      IP_topic2str = IP_topic2;
    LastRoll_topic2  = doc["LastRoll"];
      LastRoll_topic2str  = LastRoll_topic2;
    Uptime_topic2    = doc["UpTime (h)"];
    LastPicture_topic2  = doc["Last Picture"];
    Temp_topic2      = doc["Temp (°C)"];

    Serial.print(" Topic2 Device    = ");    Serial.println(Device_topic2str);
    Serial.print(" Topic2 Version   = ");    Serial.println(Version_topic2str);
    Serial.print(" Topic2 RSSI      = ");    Serial.println(RSSI_topic2);
    Serial.print(" Topic2 IP        = ");    Serial.println(IP_topic2str);
    Serial.print(" Topic2 LastRoll  = ");    Serial.println(LastRoll_topic2str);
    Serial.print(" Topic2 Uptime    = ");    Serial.println(Uptime_topic2);
    Serial.print(" Topic2 Temp      = ");    Serial.println(Temp_topic2);
    Serial.print(" Topic2 LastPic   = ");    Serial.println(LastPicture_topic2);
    Serial.println();

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
      client.subscribe(TOPIC2);                             // Subscribe to MQTT  
        Serial.print("Subscribing to: "); Serial.println(TOPIC2);
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
  
  swversion = (swversion.substring((swversion.indexOf(".")), (swversion.lastIndexOf("\\")) + 1))+" "+__DATE__+" "+__TIME__;   
   Serial.print("SW version: ");
   Serial.println(swversion);
     
  setup_wifi();                                       // Connect to network

  Serial.println("Broker MQTT setting server.. ");	
    client.setServer(BROKER_MQTT, 1883);              // MQTT port, unsecure

  Serial.println("Starting timeclient server.. "); 	
    timeClient.begin();                               // Initialize a NTPClient to get time 

  Serial.println("Initialize MQTT callback routine.. "); 	
    client.setCallback(callback);                     // Initialize the callback routine

  Serial.println("Declaring Thinger variables.. ");
    thing["data"] >> [](pson& out){  
     // Add the values and the corresponding code
      out["Device_topic0"]    = Device_topic0;
      out["Version_topic0"]   = Version_topic0;
      out["RSSI_topic0"]      = RSSI_topic0;
      out["IP_topic0"]        = IP_topic0;
      out["LastRoll_topic0"]  = LastRoll_topic0;
      out["Uptime_topic0"]    = Uptime_topic0;

      out["Device_topic1"]    = Device_topic1str;
      out["Version_topic1"]   = Version_topic1str;
      out["RSSI_topic1"]      = RSSI_topic1;
      out["IP_topic1"]        = IP_topic1str;
      out["LastRoll_topic1"]  = LastRoll_topic1str;
      out["Uptime_topic1"]    = Uptime_topic1;
      out["Temp"]             = Temp_topic1;

      out["Device_topic2"]    = Device_topic2str;
      out["Version_topic2"]   = Version_topic2str;
      out["RSSI_topic2"]      = RSSI_topic2;
      out["IP_topic2"]        = IP_topic2str;
      out["LastRoll_topic2"]  = LastRoll_topic2str;
      out["Uptime_topic2"]    = Uptime_topic2;
      out["Temp_topic2"]      = Temp_topic2;
      out["LastPic_topic2"]   = LastPicture_topic2;
      }; 

}

/*+--------------------------------------------------------------------------------------+
 *| main loop                                                                            |
 *+--------------------------------------------------------------------------------------+ */
 
void loop() {

  unsigned long currentMillis = millis();             // capture the latest value of millis() 
  uptime = millis()/3600000;                          // Update uptime 

  if (!client.connected())                            // Reconnect if connection to MQTT is lost
  {    reconnect();      }

  
  if (currentMillis - loop2 >= 30*1000) {            // Gateway device health   
   Serial.println("Loop Main: Start");

    Device_topic0     = "Suindara";
    Version_topic0    = swversion;
    RSSI_topic0       = WiFi.RSSI();
    IP_topic0         = WiFi.localIP().toString();
    LastRoll_topic0   = DateAndTime();
    Uptime_topic0     = uptime;
     
    Serial.print(" Topic0 Device    = ");    Serial.println(Device_topic0);
    Serial.print(" Topic0 Version   = ");    Serial.println(Version_topic0);
    Serial.print(" Topic0 RSSI      = ");    Serial.println(RSSI_topic0);
    Serial.print(" Topic0 IP        = ");    Serial.println(IP_topic0);
    Serial.print(" Topic0 LastRoll  = ");    Serial.println(LastRoll_topic0);
    Serial.print(" Topic0 Uptime    = ");    Serial.println(Uptime_topic0);
    Serial.println();

    Serial.print(" Topic1 Device    = ");    Serial.println(Device_topic1str);
    Serial.print(" Topic1 Version   = ");    Serial.println(Version_topic1str);
    Serial.print(" Topic1 RSSI      = ");    Serial.println(RSSI_topic1);
    Serial.print(" Topic1 IP        = ");    Serial.println(IP_topic1str);
    Serial.print(" Topic1 LastRoll  = ");    Serial.println(LastRoll_topic1str);
    Serial.print(" Topic1 Uptime    = ");    Serial.println(Uptime_topic1);
    Serial.print(" Topic1 Temp      = ");    Serial.println(Temp_topic1);
    Serial.println();

    Serial.print(" Topic2 Device    = ");    Serial.println(Device_topic2str);
    Serial.print(" Topic2 Version   = ");    Serial.println(Version_topic2str);
    Serial.print(" Topic2 RSSI      = ");    Serial.println(RSSI_topic2);
    Serial.print(" Topic2 IP        = ");    Serial.println(IP_topic2str);
    Serial.print(" Topic2 LastRoll  = ");    Serial.println(LastRoll_topic2str);
    Serial.print(" Topic2 Uptime    = ");    Serial.println(Uptime_topic2);
    Serial.print(" Topic2 Temp      = ");    Serial.println(Temp_topic2);
    Serial.print(" Topic2 LastPic   = ");    Serial.println(LastPicture_topic2);
    Serial.println();

    Serial.println("Loop Main: End");
    loop2 = currentMillis;
  }      

  thing.handle();                                 // Thinger  

  client.loop();                                  // MQTT
 
}



