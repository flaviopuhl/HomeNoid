/*
   _______  _______  ______    ___   _______  __   __  _______ 
  |       ||       ||    _ |  |   | |       ||  |_|  ||   _   |
  |  _____||    ___||   | ||  |   | |    ___||       ||  |_|  |
  | |_____ |   |___ |   |_||_ |   | |   |___ |       ||       |
  |_____  ||    ___||    __  ||   | |    ___||       ||       |
   _____| ||   |___ |   |  | ||   | |   |___ | ||_|| ||   _   |
  |_______||_______||___|  |_||___| |_______||_|   |_||__| |__|
  
 Name:     Seriema 
 Date:     DEC 2021
 Author:   Flavio L Puhl Jr <flavio_puhl@hotmail.com> 
 GIT:      
 About:    Weather forecast and temperature measurement as part of HomeNoid Project
           With OWM Api + DS18B20 temperature sensor + MQTT publsih (eclipse broker) 
 
Update comments                                      
+-----------------------------------------------------+------------------+---------------+
|               Feature added                         |     Version      |      Date     |
+-----------------------------------------------------+------------------+---------------+
| Initial Release based on IOT_Weather 105            |      1.0.0       |     DEC/21    |
|                                                     |                  |               |
|                                                     |                  |               |
+-----------------------------------------------------+------------------+---------------+


Library versions                                       
+-----------------------------------------+------------------+-------------------------- +
|       Library                           |     Version      |          Creator          |
+-----------------------------------------+------------------+-------------------------- +
| PubSubClient                            |      @^2.8       |        knolleary          |
|	ArduinoJson                             |      @^6.18.5    |        bblanchon          |
|	NTPClient                               |      @^3.1.0     |        arduino-libraries  |  
|	OneWire                                 |      @^2.3.6     |        paulstoffregen     |                           
|	DallasTemperature                       |      @^3.9.1     |        milesburton        |
|	ESP8266 ... driver for SSD1306 displays |      @^4.2.1     |        thingpulse         |
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
| RAM:   [====      ]  37.9% (used 31040 bytes from 81920 bytes)                         |
| Flash: [====      ]  37.3% (used 389077 bytes from 1044464 bytes)                      |
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

#include <ESP8266HTTPClient.h>                         // API access

#include <OneWire.h>                                   //DS18B20
#include <DallasTemperature.h>                         //DS18B20

#include <Wire.h>                                      // Only needed for Arduino 1.6.5 and earlier
#include "SSD1306Wire.h"                               // legacy: #include "SSD1306.h"

/*+--------------------------------------------------------------------------------------+
 *| Constants declaration                                                                |
 *+--------------------------------------------------------------------------------------+ */
 
const char *ssid =  "here your wifi ID";                 // name of your WiFi network
const char *password =  "here your wifi password";                   // password of the WiFi network

const char *ID = "SeriemaDev";                        // Name of our device, must be unique
const char *TOPIC = "Seriema/data";                   // Topic to subcribe to
//const char* BROKER_MQTT = "mqtt.eclipseprojects.io";  // MQTT Cloud Broker URL
const char* BROKER_MQTT = "broker.hivemq.com";

String swversion = __FILE__;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

WiFiClient wclient;
PubSubClient client(wclient);                         // Setup MQTT client

const int oneWireBus = 14;                            // GPIO where the DS18B20 is connected to
OneWire oneWire(oneWireBus);                          // Setup a oneWire instance to communicate with any OneWire devices
DallasTemperature sensors(&oneWire);                  // Pass our oneWire reference to Dallas Temperature sensor 

SSD1306Wire display(0x3c, D2, D1);                    // OLED Display

/*+--------------------------------------------------------------------------------------+
 *| Global Variables                                                                     |
 *+--------------------------------------------------------------------------------------+ */

  unsigned long loop1 = 0;                             // stores the value of millis() in each iteration of loop()
  unsigned long loop2 = 0;
  unsigned long loop3 = 0;

  float uptime = 0;
  int displayCounter = 0;
 
  long current_dt_long_global = 1639510014;
  float current_temp_float_global = 23.11;
  float current_feels_like_float_global = 23.88;
  
  long daily_0_dt_long_global = 1639580400;
  float daily_0_temp_day_float_global = 28.23;
  float daily_0_temp_min_float_global = 21.32;
  float daily_0_temp_max_float_global = 28.77;
    
  long daily_1_dt_long_global = 1639580400;
  float daily_1_temp_day_float_global = 28.23;
  float daily_1_temp_min_float_global = 21.32;
  float daily_1_temp_max_float_global = 28.77;

  long daily_2_dt_long_global = 1639666800;
  float daily_2_temp_day_float_global = 27.86;
  float daily_2_temp_min_float_global = 21.91;
  float daily_2_temp_max_float_global = 27.86;


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
  
  while (!client.connected()) {                       /* Loop until we're reconnected */
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(ID)) {
      Serial.println("connected");
      Serial.print("Publishing to: ");
      Serial.println(TOPIC);
      Serial.println('\n');

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
 *| Get Weather Forecast from OpenWeatherMap API                                         |
 *+--------------------------------------------------------------------------------------+ */

void WeatherForecast(){

//HTTPClient http;                                              //Declare an object of class HTTPClient
                                                                // Specify OWM API request destination
//http.begin("http://api.openweathermap.org/data/2.5/onecall?lat=-30.0331&lon=-51.23&exclude=hourly,minutely&units=metric&appid=b4d2a60b9952e3dd9e52a1f1196cabe6");

WiFiClient client;                                            // Works for Plataformio
  HTTPClient http;
  http.begin(client, "http://api.openweathermap.org/data/2.5/onecall?lat=-30.0331&lon=-51.23&exclude=hourly,minutely&units=metric&appid=b4d2a60b9952e3dd9e52a1f1196cabe6");

int httpCode = http.GET();                                    // Send the request
 
  if (httpCode > 0) {                                         // Check the returning code
   String OWMpayload = http.getString();                      // Get the request response payload
      //Serial.println("input");                              // for debug only
      //Serial.println(OWMpayload);
                                                              
  DynamicJsonDocument docdoc(6144);                           // Code from Arduino JSON assistant ( https://arduinojson.org/v6/assistant/ )

  DeserializationError error = deserializeJson(docdoc, OWMpayload);

    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      return;
    }
  
  JsonObject current = docdoc["current"];
    current_dt_long_global = current["dt"]; // 1639510014
    current_temp_float_global = current["temp"]; // 23.11
    current_feels_like_float_global = current["feels_like"]; // 23.88
  
  JsonArray daily = docdoc["daily"];

    JsonObject daily_0 = daily[0];
    daily_0_dt_long_global = daily_0["dt"]; // 1639580400
    
    JsonObject daily_0_temp = daily_0["temp"];
    daily_0_temp_day_float_global = daily_0_temp["day"]; // 28.23
    daily_0_temp_min_float_global = daily_0_temp["min"]; // 21.32
    daily_0_temp_max_float_global = daily_0_temp["max"]; // 28.77
    
    JsonObject daily_1 = daily[1];
    daily_1_dt_long_global = daily_1["dt"]; // 1639580400
    
    JsonObject daily_1_temp = daily_1["temp"];
    daily_1_temp_day_float_global = daily_1_temp["day"]; // 28.23
    daily_1_temp_min_float_global = daily_1_temp["min"]; // 21.32
    daily_1_temp_max_float_global = daily_1_temp["max"]; // 28.77

    JsonObject daily_2 = daily[2];
    daily_2_dt_long_global = daily_2["dt"]; // 1639666800

    JsonObject daily_2_temp = daily_2["temp"];
    daily_2_temp_day_float_global = daily_2_temp["day"]; // 27.86
    daily_2_temp_min_float_global = daily_2_temp["min"]; // 21.91
    daily_2_temp_max_float_global = daily_2_temp["max"]; // 27.86 
  }


}


/*+--------------------------------------------------------------------------------------+
 *| Get Temperature from DS18B20                                                         |
 *+--------------------------------------------------------------------------------------+ */

float getTemp(){

  String string_tempC = "";
  float float_tempC = 0;
  
  sensors.requestTemperatures();                            /* get Temperature from DS18B20 */
  float_tempC = sensors.getTempCByIndex(0);
     
  return float_tempC;
}


/*+--------------------------------------------------------------------------------------+
 *| Serialize JSON and publish MQTT                                                      |
 *+--------------------------------------------------------------------------------------+ */

void SerializeAndPublish() {

  if (!client.connected())                            /* Reconnect if connection to MQTT is lost */
  {    reconnect();      }

  client.loop();                                      /* MQTT */

  char buff[10];                                      /* Buffer to allocate decimal to string conversion */
  char buffer[256];                                   /* JSON serialization */
  
    StaticJsonDocument<256> doc;                      /* See ArduinoJson Assistant V6 */
    
      doc["Device"] = "Seriema";
      doc["Version"] = swversion;
      doc["RSSI (db)"] = WiFi.RSSI();
      doc["IP"] = WiFi.localIP();
      doc["LastRoll"] = DateAndTime();
      doc["UpTime (h)"] = uptime;
      doc["Temp (°C)"] = dtostrf(getTemp(), 2, 1, buff);
    
    serializeJson(doc, buffer);
      Serial.println("JSON Payload:");
    serializeJsonPretty(doc, Serial);                 /* Print JSON payload on Serial port */        
      Serial.println("");
                         
      Serial.println("Sending message to MQTT topic");
    client.publish(TOPIC, buffer);                    /* Publish data to MQTT Broker */
      Serial.println("");

}


/*+--------------------------------------------------------------------------------------+
 *| Screen #1 - Shows current day forecast                                               |
 *+--------------------------------------------------------------------------------------+ */

void Screen_Actual(){
 
  char buff[10];

  // Current temperature forecast
  dtostrf(current_temp_float_global, 2, 1, buff);         //2 is mininum width, 1 is precision
    String current_temp_string_local = buff;
  
  // Current feels like temperature forecast
  dtostrf(current_feels_like_float_global, 2, 1, buff);   //2 is mininum width, 1 is precision
    String current_feels_like_string_local = buff;

  // Current date & time of forecast
  time_t current_dt_long_local = current_dt_long_global-10800;
  struct tm * tm = localtime(&current_dt_long_local);  
  
  char current_date_string_local[22];                                                  
    strftime(current_date_string_local, sizeof(current_date_string_local), "%d%b", tm);    // https://www.cplusplus.com/reference/ctime/strftime/

  char current_time_string_local[22];                                                     
    strftime(current_time_string_local, sizeof(current_time_string_local), "%H-%M", tm);    // https://www.cplusplus.com/reference/ctime/strftime/

  display.clear();

    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_10);
      display.drawString(0, 0, current_time_string_local);

    display.setTextAlignment(TEXT_ALIGN_RIGHT);
    display.setFont(ArialMT_Plain_10);
      display.drawString(128, 0, current_date_string_local); 
        
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_10);
        display.drawString(0, 13, "EXTERNO");
        display.drawString(90, 13, "FEELS");
      
    display.setFont(Dialog_plain_30);           // Show ext temp
      display.drawString(0, 31, current_temp_string_local);
      
    display.setFont(Dialog_plain_16);
      display.drawString(68, 43, "°C");                         

    display.setTextAlignment(TEXT_ALIGN_RIGHT);
      display.setFont(Dialog_plain_16);
      display.drawString(128, 25, current_feels_like_string_local+ " ºC");
      
display.display();

}

/*+--------------------------------------------------------------------------------------+
 *| Screen #2 - Shows temperature from sensor                                            |
 *+--------------------------------------------------------------------------------------+ */

void Screen_TempInt(){

// Current date & time of forecast
  time_t current_dt_long_local = current_dt_long_global-10800;
  struct tm * tm = localtime(&current_dt_long_local);  
  
  char current_date_string_local[22];                                                  
    strftime(current_date_string_local, sizeof(current_date_string_local), "%d%b", tm);    // https://www.cplusplus.com/reference/ctime/strftime/

  char current_time_string_local[22];                                                     
    strftime(current_time_string_local, sizeof(current_time_string_local), "%H-%M", tm);    // https://www.cplusplus.com/reference/ctime/strftime/

  char buff[10];
    dtostrf(getTemp(), 2, 1, buff);  //2 is mininum width, 1 is precision
    String string_tempC = buff;

display.clear();

  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
    display.drawString(0, 0, current_time_string_local);

  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.setFont(ArialMT_Plain_10);
    display.drawString(128, 0, current_date_string_local);

  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
    display.drawString(0, 13, "INTERNO");

  display.setFont(Dialog_plain_40);
  display.drawString(10, 25, string_tempC);
  display.setFont(Dialog_plain_16);
    display.drawString(100, 45, "°C");      
     
display.display();

}


/*+--------------------------------------------------------------------------------------+
 *| Screen #3 - Shows 2 days forecast                                                    |
 *+--------------------------------------------------------------------------------------+ */

void Screen_Forecast(){
 
 char buff[10];
   
  // Day1 Min temperature forecast
  dtostrf(daily_1_temp_min_float_global, 2, 0, buff);         //2 is mininum width, 0 is precision
    String daily_1_temp_min_string_local = buff;
  // Day1 Max temperature forecast
  dtostrf(daily_1_temp_max_float_global, 2, 0, buff);         //2 is mininum width, 0 is precision
    String daily_1_temp_max_string_local = buff;

  // Day2 Min temperature forecast
  dtostrf(daily_2_temp_min_float_global, 2, 0, buff);         //2 is mininum width, 0 is precision
    String daily_2_temp_min_string_local = buff;
  // Day2 Max temperature forecast
  dtostrf(daily_2_temp_max_float_global, 2, 0, buff);         //2 is mininum width, 0 is precision
    String daily_2_temp_max_string_local = buff;

  // Current date & time of forecast
  time_t current_dt_long_local = current_dt_long_global-10800;
  struct tm * tm = localtime(&current_dt_long_local);  
  
  char current_date_string_local[22];                                                  
    strftime(current_date_string_local, sizeof(current_date_string_local), "%d%b", tm);    // https://www.cplusplus.com/reference/ctime/strftime/

  char current_time_string_local[22];                                                     
    strftime(current_time_string_local, sizeof(current_time_string_local), "%H-%M", tm);    // https://www.cplusplus.com/reference/ctime/strftime/

  // Day1 date forecast
  time_t daily_1_dt_long_local = daily_1_dt_long_global-10800;
  struct tm * tm1 = localtime(&daily_1_dt_long_local);

  char daily_1_dt_string_local[22];                                                  
    strftime(daily_1_dt_string_local, sizeof(daily_1_dt_string_local), "%d%b", tm1);    // https://www.cplusplus.com/reference/ctime/strftime/

   // Day2 date forecast
  time_t daily_2_dt_long_local = daily_2_dt_long_global-10800;
  struct tm * tm2 = localtime(&daily_2_dt_long_local);

  char daily_2_dt_string_local[22];                                                  
    strftime(daily_2_dt_string_local, sizeof(daily_2_dt_string_local), "%d%b", tm2);    // https://www.cplusplus.com/reference/ctime/strftime/

display.clear();

  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
    display.drawString(0, 0, current_time_string_local);

  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.setFont(ArialMT_Plain_10);
    display.drawString(128, 0, current_date_string_local);
    
	//display.drawLine(64, 20, 64, 50);
      
  display.setFont(Dialog_plain_12);
	display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.drawString(0, 32, "Min: ");
		display.drawString(0, 47, "Max: "); 					
        
  display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.drawString(56, 15, daily_1_dt_string_local);
    display.drawString(56, 32, daily_1_temp_min_string_local+"°C");
    display.drawString(56, 47, daily_1_temp_max_string_local+"°C");          
           
    display.setTextAlignment(TEXT_ALIGN_RIGHT);
    display.drawString(128, 15, daily_2_dt_string_local);
    display.drawString(128, 32, daily_2_temp_min_string_local+"°C");
    display.drawString(128, 47, daily_2_temp_max_string_local+"°C");       

display.display();

}


/*+--------------------------------------------------------------------------------------+
 *| Setup                                                                                |
 *+--------------------------------------------------------------------------------------+ */
 
void setup() {
  Serial.begin(115200);                               /* Start serial communication at 115200 baud */
    delay(100);

  sensors.begin();                                    /* Start the DS18B20 sensor */

  display.init();                                     /* OLED display */
  display.flipScreenVertically();
  display.clear();    
  
  swversion = (swversion.substring((swversion.indexOf(".")), (swversion.lastIndexOf("\\")) + 1))+" "+__DATE__+" "+__TIME__;   
   Serial.print("SW version: ");
   Serial.println(swversion);

      display.setFont(ArialMT_Plain_10);
      display.setTextAlignment(TEXT_ALIGN_LEFT);
      display.drawString(0, 0, swversion);            /* Display sw version */
      display.display();
      display.drawString(0, 13, "Starting WiFi .... ");
      display.display();
     
  setup_wifi();                                       /* Connect to network */
    display.drawString(0, 26, "Wifi connected .... ");
    display.display();
  
  Serial.println("Broker MQTT setting server.. ");	
  client.setServer(BROKER_MQTT, 1883);                /* MQTT port, unsecure */

  Serial.println("Starting timeclient server.. "); 	
  timeClient.begin();                                 /* Initialize a NTPClient to get time */

  delay(2000);
    display.clear();
  
  Serial.println("Initial forecast acquisition.. ");  
    WeatherForecast();
  
  Serial.print("Initial MQTT publish .. "); 
    SerializeAndPublish();  

}


/*+--------------------------------------------------------------------------------------+
 *| main loop                                                                            |
 *+--------------------------------------------------------------------------------------+ */
 
void loop() {  

  unsigned long currentMillis = millis();             /* capture the latest value of millis() */
  uptime = millis()/3600000;                          /* Update uptime */
    
    
    /*------ Loop 03 sec ------*/
     
    if (currentMillis - loop1 >= 3*1000) {          
        Serial.println("Loop Screen: Start");
          
          displayCounter++;                           /* Show and cycle screen every 3sec */
        
            if(displayCounter==0){Screen_Actual();}
            if(displayCounter==1){Screen_TempInt(); }
            if(displayCounter==2){Screen_Forecast(); displayCounter=-1;}
        
        Serial.println("Loop Screen: End");
        loop1 = currentMillis;
    }


    /*------ Loop 30 sec ------*/
     
    if (currentMillis - loop2 >= 30*1000) {          
        Serial.println("Loop MQTT: Start");
         
        SerializeAndPublish();                        /* Serialize and Publish data */

        Serial.println("Loop MQTT: End");
        loop2 = currentMillis;
    }
    
   
    /*------ Loop 300 sec ------*/

    if (currentMillis - loop3 >= 5*60*1000) {                                     
        Serial.println("Loop OWM: Start");
         
        WeatherForecast();                            /* Get WeatherOpenMap */
     
        Serial.println("Loop OWM: End");
        loop3 = currentMillis;
    }

  }
