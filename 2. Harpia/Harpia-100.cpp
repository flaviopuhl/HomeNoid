/*
   __   __  _______  ______    _______  ___   _______ 
  |  | |  ||   _   ||    _ |  |       ||   | |   _   |
  |  |_|  ||  |_|  ||   | ||  |    _  ||   | |  |_|  |
  |       ||       ||   |_||_ |   |_| ||   | |       |
  |       ||       ||    __  ||    ___||   | |       |
  |   _   ||   _   ||   |  | ||   |    |   | |   _   |
  |__| |__||__| |__||___|  |_||___|    |___| |__| |__|

 Name:     Harpia 
 Date:     DEC 2021
 Author:   Flavio L Puhl Jr <flavio_puhl@hotmail.com> 
 GIT:      
 About:    Balcony image capture and temperature meas as part of HomeNoid Project
           With BMP180 sensor + MQTT publsih (eclipse broker) + Google firebase
 
Update comments                                      
+-----------------------------------------------------+------------------+---------------+
|               Feature added                         |     Version      |      Date     |
+-----------------------------------------------------+------------------+---------------+
| Initial Release based on Hapia_eval9                |      1.0.0       |     DEC/21    |
|                                                     |                  |               |
|                                                     |                  |               |
+-----------------------------------------------------+------------------+---------------+


Library versions                                       
+-----------------------------------------+------------------+-------------------------- +
|       Library                           |     Version      |          Creator          |
+-----------------------------------------+------------------+-------------------------- +
|	PubSubClient                            |     @^2.8        | knolleary           |
|	ArduinoJson                             |     @^6.18.5     | bblanchon           |
|	NTPClient                               |     @^3.1.0      | arduino-libraries   |
|	Firebase Library for ESP8266 and ESP32  |     @^2.7.5      | mobizt              |
|	LittleFS_esp32                          |     @^1.0.6      | lorol               |
|	OneWire                                 |     @^2.3.6      | paulstoffregen      |
|	DallasTemperature                       |     @^3.9.1      | milesburton         |
+-----------------------------------------+------------------+-------------------------- +

Upload settings 
+----------------------------------------------------------------------------------------+
| PLATFORM: Espressif 32 (3.3.0) > AI Thinker ESP32-CAM                                  |
| HARDWARE: ESP32 240MHz, 320KB RAM, 4MB Flash                                           |
| PACKAGES:                                                                              |
|  - framework-arduinoespressif32 3.10006.210326 (1.0.6)                                 |
|  - tool-esptoolpy 1.30100.210531 (3.1.0)                                               |
|  - toolchain-xtensa32 2.50200.97 (5.2.0)                                               |
|                                                                                        |
| RAM:   [==        ]  18.2% (used 59496 bytes from 327680 bytes)                        |
| Flash: [====      ]  40.7% (used 1279730 bytes from 3145728 bytes)                     |
+----------------------------------------------------------------------------------------+

*/

/*+--------------------------------------------------------------------------------------+
 *| Libraries                                                                            |
 *+--------------------------------------------------------------------------------------+ */

// Libraries built into IDE

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

#include "esp_camera.h"
#include "Arduino.h"

#include "img_converters.h"                                 // Txt overlay testing
#include "esp_timer.h"
#include "img_converters.h"
#include "fb_gfx.h"
#include "fd_forward.h"
#include "fr_forward.h"

#include "SD_MMC.h"                                         // SD Card ESP32
#include "soc/soc.h"                                        // Disable brownout problems
#include "soc/rtc_cntl_reg.h"                               // Disable brownout problems
#include "driver/rtc_io.h"
#include <EEPROM.h>                                         // read and write from flash memory
//#include <SPIFFS.h>
#include <LittleFS.h>
#include <FS.h>
#include <Firebase_ESP_Client.h>                            //Firebase
#include "addons/TokenHelper.h"                             // Firebase Provide the token generation process info.

#include <OneWire.h>                                        //DS18B20
#include <DallasTemperature.h>                              //DS18B20

/*+--------------------------------------------------------------------------------------+
 *| Constants declaration                                                                |
 *+--------------------------------------------------------------------------------------+ */

// define the number of bytes you want to access
#define EEPROM_SIZE 512

// Pin definition for CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

const char *ssid =  "here your wifi ID";                         // name of your WiFi network
const char *password =  "here your wifi password";                           // password of the WiFi network

const char *ID = "HarpiaDev";                                 // Name of our device, must be unique
const char *TOPIC = "Harpia/data";                            // Topic to subcribe to
const char* BROKER_MQTT = "broker.hivemq.com";                // MQTT Cloud Broker URL
//const char* BROKER_MQTT = "mqtt.eclipseprojects.io";          // MQTT Cloud Broker URL

#define API_KEY "here your wifi Firebase API Key"     // Firebase: Define the API Key 
#define USER_EMAIL "here your wifi Firebase email"                   // Firebase: Define the user Email 
#define USER_PASSWORD "here your wifi Firebase user password"                           // Firebase: Define password 
#define STORAGE_BUCKET_ID "here your wifi Firebase bucket"            // Firebase: Define the Firebase storage bucket ID e.g bucket-name.appspot.com 
#define FILE_PHOTO_FS "/data/Harpia_photo.jpg"                // Photo File Name to save in File System

boolean takeNewPhoto = true;
bool taskCompleted = false;

FirebaseData fbdo;                                            // Firebase: Define Firebase Data object
FirebaseAuth auth;                                            // Firebase
FirebaseConfig configF;                                       // Firebase

String swversion = __FILE__;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

WiFiClient wclient;
PubSubClient client(wclient);                                 // Setup MQTT client

const int oneWireBus = 33;                                    // GPIO where the DS18B20 is connected to
OneWire oneWire(oneWireBus);                                  // Setup a oneWire instance to communicate with any OneWire devices
DallasTemperature sensors(&oneWire);                          // Pass our oneWire reference to Dallas Temperature sensor

/*+--------------------------------------------------------------------------------------+
 *| Global Variables                                                                     |
 *+--------------------------------------------------------------------------------------+ */

  unsigned long loop1 = 0;                                    // stores the value of millis() in each iteration of loop()
  unsigned long loop2 = 0;  
  unsigned long loop3 = 0;

  float uptime = 0;

  String fileName = "empty";                                  // filename that will be sent on MQTT

  int cameraErrorCnt = 0;
  int cameraErrorMax = 6;

/*+--------------------------------------------------------------------------------------+
 *| Text Overlay                                                                         |
 *+--------------------------------------------------------------------------------------+ */

static void rgb_print(dl_matrix3du_t *image_matrix, uint32_t color, const char * str){
    fb_data_t fb;
    fb.width = image_matrix->w;
    fb.height = image_matrix->h;
    fb.data = image_matrix->item;
    fb.bytes_per_pixel = 3;
    fb.format = FB_BGR888;
    //fb_gfx_print(&fb, (fb.width - (strlen(str) * 14)) / 2, 10, color, str);
    fb_gfx_print(&fb, (fb.width - (strlen(str) * 14)) / 2, fb.height-20, color, str);
}

/*+--------------------------------------------------------------------------------------+
 *| Initiate DS18B20                                                                     |
 *+--------------------------------------------------------------------------------------+ */

void initDS18B20(){

  int t = 0;
  int available = 0;
  float float_tempCinit = 0;

  do{

  log_i("DS18B20 init trial: %i", t);
  sensors.begin();
    delay(500);
    available = sensors.getDeviceCount();
      log_i("Number of I2C devices found on the bus: %i", available);

  t++;
  } while (available == 0 && t<=20);

  if(available >=1 ){
  delay(500);

  sensors.requestTemperatures();                            // get Temperature from DS18B20 
  delay(1000);
    float_tempCinit = sensors.getTempCByIndex(0);
    log_i("Initial Temperature request: %f", float_tempCinit);
  } else {
    log_i("Initial Temperature request: failed");
  }
  
}

/*+--------------------------------------------------------------------------------------+
 *| Get Temperature from DS18B20                                                         |
 *+--------------------------------------------------------------------------------------+ */

float getTemp(){
  
  delay(500);
   String string_tempC = "";
   float float_tempC = 0;

    sensors.requestTemperatures();                            // get Temperature from DS18B20 
      delay(1000);
      float_tempC = sensors.getTempCByIndex(0);
      //log_i("Temperature: %f", float_tempC);
     
  return float_tempC;
}


/*+--------------------------------------------------------------------------------------+
 *| Initiate File System                                                                 |
 *+--------------------------------------------------------------------------------------+ */

void initFS(){

  if (!SPIFFS.begin(true)) {
    log_i("An Error has occurred while mounting FS");
      ESP.restart();
  }  else {
    log_i("File System mounted successfully");
      delay(500);
  }
}

/*+--------------------------------------------------------------------------------------+
 *| Initiate SDCARD                                                                      |
 *+--------------------------------------------------------------------------------------+ */
  
  void initSDCARD(){

    //if(!SD_MMC.begin()){
    if(!SD_MMC.begin("/sdcard", true)){               // note: ("/sdcard", true) = 1 wire - see: https://www.reddit.com/r/esp32/comments/d71es9/a_breakdown_of_my_experience_trying_to_talk_to_an/
      log_i("SD Card Mount Failed");
      ESP.restart();
      return;
    } else {
      log_i("SD Card Mount successfull");
    }
  

    uint8_t cardType = SD_MMC.cardType();

    if(cardType == CARD_NONE){
      log_i("No SD Card attached");
      ESP.restart();
        return;
    } else {
      log_i("SD Card attached");
    }

  }

/*+--------------------------------------------------------------------------------------+
 *| Initiate EEPROM                                                                      |
 *+--------------------------------------------------------------------------------------+ */
  
void initEEPROM(){

  if (!EEPROM.begin(EEPROM_SIZE)){                                  // Initialize EEPROM with predefined size
      log_i("failed to initialise EEPROM...");
      ESP.restart();
    } else {
      log_i("Success to initialise EEPROM...");
    }

}
/*+--------------------------------------------------------------------------------------+
 *| Initiate CAMERA                                                            |
 *+--------------------------------------------------------------------------------------+ */
  
  void initCAMERA(){

    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 10000000;                             // https://github.com/espressif/esp32-camera/issues/93
    config.pixel_format = PIXFORMAT_JPEG;                       // YUV422,GRAYSCALE,RGB565,JPEG
    
    if(psramFound()){                                           // https://github.com/espressif/esp-who/issues/83
      log_i("PSRAM found");
      //config.frame_size = FRAMESIZE_UXGA; 						        // FRAMESIZE_ + QVGA  ( 320 x 240 )
      config.frame_size = FRAMESIZE_VGA; 						            // FRAMESIZE_ + QVGA  ( 320 x 240 ) 
      config.jpeg_quality = 10;                                 //              CIF   ( 352 x 288)
      config.fb_count = 1;                                      //              VGA   ( 640 x 480 )
    } else {                                                    //              SVGA  ( 800 x 600 )
      log_i("PSRAM not found");                                 //              XGA   ( 1024 x 768 )
      config.frame_size = FRAMESIZE_SVGA;                       //              SXGA  ( 1280 x 1024 )
      config.jpeg_quality = 12;                                 //              UXGA  ( 1600 x 1200 )
      config.fb_count = 1;
    }

    if(psramInit()){
      log_i("PSRAM initiated");
    } else {
      log_i("PSRAM initiation failed");
    }


    int tt = 0;
    esp_err_t err;
    do{
      err = esp_camera_init(&config);
        if (err != ESP_OK) {
          log_i("Camera init failed with error 0x%x", err);
          log_i("Init trial %i", tt);        
          tt++;
        } else {
          log_i("Camera init successfull");
        }
    } while (err != ESP_OK && tt<=20);

    if(tt >= 20){
      delay(500);
      ESP.restart();
      }

  }

/*+--------------------------------------------------------------------------------------+
 *| Connect to WiFi network                                                              |
 *+--------------------------------------------------------------------------------------+ */

void setup_wifi() {

  log_i("Connecting to %s",ssid);
    WiFi.mode(WIFI_STA);                                     // Setup ESP in client mode
    
    WiFi.begin(ssid, password);                              // Connect to network

    int wait_passes = 0;
    while (WiFi.status() != WL_CONNECTED) {                  // Wait for connection
      delay(500);
      log_i(".");
      if (++wait_passes >= 20) { ESP.restart(); }            // Restart in case of no wifi connection   
    }

  log_i("WiFi connected");
  

}

 
/*+--------------------------------------------------------------------------------------+
 *| Reconnect to MQTT client                                                             |
 *+--------------------------------------------------------------------------------------+ */
 
void reconnect() {
  
  while (!client.connected()) {                               // Loop until we're reconnected 
    log_i("Attempting MQTT connection...");
    if (client.connect(ID)) {
      log_i("MQTT broker connected");
      log_i("Publishing to: %s",TOPIC);
    } else {
      log_i(" try again in 5 seconds");
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
 *| Serialize JSON and publish MQTT                                                      |
 *+--------------------------------------------------------------------------------------+ */

void SerializeAndPublish() {

  if (!client.connected())                            // Reconnect if connection to MQTT is lost 
  {    reconnect();      }

  client.loop();                                      // MQTT 

  char buff[10];                                      // Buffer to allocate decimal to string conversion 
  char buffer[256];                                   // JSON serialization 
  
    StaticJsonDocument<256> doc;                      // See ArduinoJson Assistant V6 
    
      doc["Device"] = "Harpia";
      doc["Version"] = swversion;
      doc["RSSI (db)"] = WiFi.RSSI();
      doc["IP"] = WiFi.localIP();
      doc["LastRoll"] = DateAndTime();
      doc["UpTime (h)"] = uptime;
      doc["Last Picture"] = fileName;
      doc["Temp (°C)"] = dtostrf(getTemp(), 2, 1, buff);
    
    serializeJson(doc, buffer);
      log_i("\nJSON Payload:");
      Serial.printf("\n");
    serializeJsonPretty(doc, Serial);                 // Print JSON payload on Serial port        
      Serial.printf("\n");                  
      log_i("Sending message to MQTT topic");
    client.publish(TOPIC, buffer);                    // Publish data to MQTT Broker 

}

/*+--------------------------------------------------------------------------------------+
 *| Take a picture, save to SD Card, save to SPPIFFS, Send to firebase                   |
 *+--------------------------------------------------------------------------------------+ */
 
void takeImage(){
  
  log_i("Taking a photo...");
   
  camera_fb_t * fb = NULL;                                            // Reset camera pointer
  
  bool CamCaptureSucess = true;                                       // Boolean indicating if the picture has been taken correctly
  bool CamCaptureSize = true;

  fb = esp_camera_fb_get();                                           // Take Picture with Camera  
    if(!fb) {
      CamCaptureSucess = false;
        log_i("Camera capture failed...");

        initCAMERA();

    } else {
      CamCaptureSucess = true;
        log_i("Camera capture success...");
    } 

      // Text verlay START

        String txtOverlay = DateAndTime();                                  // https://stackoverflow.com/questions/17853988/convert-string-to-const-char-issue
        const char* txtOverlay_char = txtOverlay.c_str();

        dl_matrix3du_t *image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);
        fmt2rgb888(fb->buf, fb->len, fb->format, image_matrix->item);  
        rgb_print(image_matrix, 0x00000000, txtOverlay_char);               // https://www.littlewebhut.com/css/value_color/

        size_t _jpg_buf_len = 0;
        uint8_t * _jpg_buf = NULL;
        fmt2jpg(image_matrix->item, fb->width*fb->height*3, fb->width, fb->height, PIXFORMAT_RGB888, 90, &_jpg_buf, &_jpg_buf_len);
          //int available_PSRAM_size = ESP.getFreePsram();
          int available_PSRAM_size = ESP.getFreeHeap();
          log_i("PSRAM Size available (bytes)           : %i", available_PSRAM_size);
        dl_matrix3du_free(image_matrix);
          //int available_PSRAM_size_after = ESP.getFreePsram();
          int available_PSRAM_size_after = ESP.getFreeHeap();
          log_i("PSRAM Size available after free (bytes): %i", available_PSRAM_size_after);
      
      // Text overlay END

  if (CamCaptureSucess == true){ 

    int t = 0;
    unsigned int pic_sz = 0;
      
    File fileFS = SPIFFS.open(FILE_PHOTO_FS, FILE_WRITE);             // Save picture on FS
    log_i("Picture file name (FS): %s", FILE_PHOTO_FS);               // FS Photo file name
    // Insert the data in the photo file
    if (!fileFS) {
      log_i("Failed to open file (FS) in writing mode");
    } else {
      log_i("File (FS) open in writing mode : %s",FILE_PHOTO_FS);

        do{
        //fileFS.write(fb->buf, fb->len);                             // payload (image), payload length (works!) commented to test txt overlay
        fileFS.write(_jpg_buf, _jpg_buf_len);
        log_i("The picture has been saved in (FS) ");
        pic_sz = fileFS.size();                                     
          log_i("File Size: %i bytes | read trial: %i of 20", pic_sz, t);
          t++;
        } while (pic_sz == 0 && t <= 20);
    }

    // Close the file
    fileFS.close();

   
    if ( pic_sz > 100 ){
      log_i("Picture size is valid ... ");
        CamCaptureSize = true;
    } else {
      log_i("Picture size is not valid ... ");
        CamCaptureSize = false;
    };

      
    if(CamCaptureSize == true){

      int pictureNumber = 0;

      EEPROM.get(8,pictureNumber);                                            // Recover picture counter from EEPROM
        log_i("Current picture number counter : %i",pictureNumber);
        
        if(pictureNumber>=17280) {                                            // Increment picture counter and save to EEPROM
          EEPROM.put(8,1);                                                    // Rational
        } else {         
          pictureNumber++;                                                    // New picture each 5 min
          EEPROM.put(8,pictureNumber);                                        // Period = 2 months   
        }                                                                     // FIFO  = ( 2 months * 30 days * 24 hours * 60 min ) / 5 = 17280 
          EEPROM.commit();                                                                                                                                        

        EEPROM.get(8,pictureNumber);                                          // Recover new picture counter and use it to name the file on SD CARD
          log_i("New picture number counter : %i",pictureNumber);


        String path = "/picture" + String(pictureNumber) +".jpg";             // Path where new picture will be saved in SD Card 

        fs::FS &fs = SD_MMC;
        fileName = path.c_str();
          log_i("Picture file name (SDCARD): %s", path.c_str());
        
        File fileSDCARD = fs.open(path.c_str(), FILE_WRITE);                  // Save image on SD Card with dynamic name
          if(!fileSDCARD){
            fileName = "fail to save pic";
              log_i("Failed to open file (SDCARD) in writing mode...");
          } else {
            //fileSDCARD.write(fb->buf, fb->len); // payload (image), payload length
            fileSDCARD.write(_jpg_buf, _jpg_buf_len); // payload (image), payload length
              log_i("The picture has been saved in (SDCARD) : %s", path.c_str());
          }
          
          fileSDCARD.close();

        esp_camera_fb_return(fb);
    
    } //CamCaptureSize = true;
        
  } //CamCaptureSucess = false;

  
  if(CamCaptureSucess == true && CamCaptureSize == true) {
    if (Firebase.ready())
        {
            taskCompleted = true;
        
            log_i("Upload file to Firebase... ");

            //MIME type should be valid to avoid the download problem.
            //The file systems for flash and SD/SDMMC can be changed in FirebaseFS.h.
            if (Firebase.Storage.upload(&fbdo, 
              STORAGE_BUCKET_ID                   /* Firebase Storage bucket id */, 
              FILE_PHOTO_FS                       /* path to local file */, 
              //mem_storage_type_sd               /* memory storage type, mem_storage_type_flash and mem_storage_type_sd */, 
              mem_storage_type_flash              /* memory storage type, mem_storage_type_flash and mem_storage_type_sd */, 
              FILE_PHOTO_FS                       /* path of remote file stored in the bucket */, 
              "image/jpeg"                        /* mime type */
            )) 
                log_i("Download URL: %s\n", fbdo.downloadURL().c_str());
            else
                Serial.println(fbdo.errorReason());
        }else{
          log_i("Firebase not ready OR Picture size invalid");
        }
  }
  
  // Turns off the ESP32-CAM white on-board LED (flash) connected to GPIO 4
  //pinMode(4, OUTPUT);                   // Those lines must be commented to allow
  //digitalWrite(4, LOW);                 //  the SD card pin to be released
  //rtc_gpio_hold_en(GPIO_NUM_4);         // If not commented, SD card file saving on
                                          //  2nd loop will result in critical fault

 delay(100);

}


/*+--------------------------------------------------------------------------------------+
 *| Setup                                                                                |
 *+--------------------------------------------------------------------------------------+ */
 
void setup() {
  Serial.begin(115200);                                       // Start serial communication at 115200 baud 
    delay(5000); 
  
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); 				          //disable brownout detector

  swversion = (swversion.substring((swversion.indexOf(".")), (swversion.lastIndexOf("\\")) + 1))+" "+__DATE__+" "+__TIME__;   
   log_i("SW version: %s", swversion);

     
  setup_wifi();                                               // Connect to network 
  
  log_i("Broker MQTT setting server... ");
    client.setServer(BROKER_MQTT, 1883);                      // MQTT port, unsecure

  log_i("Starting timeclient server... "); 	
    timeClient.begin();                                       // Initialize a NTPClient to get time 

  log_i("Firebase Client v%s", FIREBASE_CLIENT_VERSION);      // Firebase

  configF.api_key = API_KEY;                       			      // Firebase: Assign the api key (required)
  auth.user.email = USER_EMAIL;                   			      // Firebase: Assign the user sign in credentials
  auth.user.password = USER_PASSWORD;
  configF.token_status_callback = tokenStatusCallback; 		    // Firebase: Assign the callback function for the long running token generation task
															                                // see addons/TokenHelper.h
  Firebase.begin(&configF, &auth);                 			      // Firebase
  Firebase.reconnectWiFi(true);                   			      // Firebase                
  
  log_i("Hard reset OV2640");
    digitalWrite(PWDN_GPIO_NUM, HIGH);                        //HIGH = PWDN  || LOW = camera PWUP
      delay(2000);
    digitalWrite(PWDN_GPIO_NUM, LOW);     
  log_i("Hard reset OV2640 ... done");

  initSDCARD();
  initFS();
  initCAMERA();
  initEEPROM();
  initDS18B20();

  log_i("Internal Total heap %d, internal Free Heap %d", ESP.getHeapSize(), ESP.getFreeHeap());
  log_i("SPIRam Total heap %d, SPIRam Free Heap %d", ESP.getPsramSize(), ESP.getFreePsram());
  log_i("ChipRevision %d, Cpu Freq %d, SDK Version %s", ESP.getChipRevision(), ESP.getCpuFreqMHz(), ESP.getSdkVersion());
  log_i("Flash Size %d, Flash Speed %d", ESP.getFlashChipSize(), ESP.getFlashChipSpeed());

  log_i("Initial image capturing ... "); 
    takeImage();
  log_i("Initial MQTT publish ... "); 
    SerializeAndPublish();  

 
}


/*+--------------------------------------------------------------------------------------+
 *| main loop                                                                            |
 *+--------------------------------------------------------------------------------------+ */

void loop() {
  
  unsigned long currentMillis = millis();             // capture the latest value of millis() 
  uptime = millis()/3600000;                          // Update uptime 

  
  if (currentMillis - loop1 >= 1*60*1000) {          

    if (WiFi.status() != WL_CONNECTED){
      setup_wifi(); 
    } else {
 
      log_i("Internal Total heap %d, internal Free Heap %d", ESP.getHeapSize(), ESP.getFreeHeap());
      log_i("SPIRam Total heap   %d, SPIRam Free Heap   %d", ESP.getPsramSize(), ESP.getFreePsram());
      initDS18B20();
      log_i("WiFi status is normal... %u millis\n", currentMillis);
    }
    
    loop1 = currentMillis;
  }


     
  if (currentMillis - loop2 >= 5*60*1000) {          
    log_i("Loop MQTT: Start");
         
      takeImage();

      SerializeAndPublish();                        // Serialize and Publish data //

      
    log_i("Loop MQTT: End");
    loop2 = currentMillis;
  }

 
}

