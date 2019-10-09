  /* MOTION - AFO sensor test - 2 Datalogger setup - Master
 *  
 *  This program is created to read out all the sensors on a preliminairy AFO to test what data can be extracted from the different signals with regard to gait phase detection.
 *  
 *  This code describes the functionality of the master. The master uses pin 27 to trigger the slave datalogger to perform a measurement. 
 *  
 *  The sample frequency is set to 20ms or 50Hz, two Bosch BNO055 sensors provide their quaternion output, acceleration output and gyro output. 
 *  The angle between the two quaternions is calculated with the quaternion dot operator, this calculates the smallest angle between the two. 
 *  Additionally five analog ouput are read out, these represent the  Force Sensitive Resistors under the instrumented AFO
 *  All these parameters are logged to a SD card. 
 *  
 *  A measurement can be triggered by sending the command 's' over a bluetooth (bluetooth classic using BluetoothSerial Lib) connection (e.g. android BTSerial app) after connection/pairing with 'ESP32_MOTION' has been made
 *  A measurement can also be triggered by pushing button C on the electronics module. A call back function monitors the connection/disconnection of clients.
 *  
 *  Whenever a new measuremnt starts, a data sample from the RTC is requested and reformatted to be used as the new log file's name: e.g. 20190923_10h_09m_48s, in this way a new 
 *  measurement has a unique identifier.
 *  
 *  The device utilizes a Neopixel LED to communicate visual feedback about it's status.
 *  An OLED module provides more detailed feedback for debugging and close range inspection.
 *  
 *  Note that before this code can be used the ESP32 core from espressiv needs to be installed, the driver for USB communication SiLabs CP2104 Needs to be installed on your PC, as well as the utilized librairies. 

   BOSCH BNO INFORMATION
   =====================
   
   Breakout boards used are the adafruit Bosch BNO055 breakouts (product ID: 3463).

   Connections BOSCH BNO055 - 1 (I2C Adress is 0x28)
   =================================================
   Connect SCL to Huzzah ESP32 SCL pin
   Connect SDA to Huzzah ESP32 SCL pin   
   Connect VDD to Huzzah ESP32 3V pin
   Connect GROUND to Huzzah ESP32 GND

   Connections BOSCH BNO055 - 2 (I2C Adress is 0x29)
   =================================================
   Connect SCL to Huzzah ESP32 SCL pin
   Connect SDA to Huzzah ESP32 SCL pin   
   Connect VDD to Huzzah ESP32 3V pin
   Connect GROUND to common ground
   Connect GROUND to Huzzah ESP32 GND

      Board layout:
         +----------+
         |         *| RST   PITCH  ROLL  HEADING
     ADR |*        *| SCL
     INT |*        *| SDA     ^            /->
     PS1 |*        *| GND     |            |
     PS0 |*        *| 3VO     Y    Z-->    \-X
         |         *| VIN
         +----------+

   SD_Card and RTC board
   =====================
   Adafruit adalogger featherwing RTC + SD was used (product ID: 2922)
   RTC interface: I2C
   SD interface: SPI + Chip Select (CS)
   SD CS was set to GPIO pin 33 (default connection between ESP32 and Adalogge CS pin)
   RTC is set with the adafruit RTC_PCF8523 set RTC example.

   OLED Display:
   =============

   Adafruit featherwing OLED (product ID:2900)
   Screen Width: 128 px
   Scree Height: 32 px
   Interface I2C

   Btn A connected to GPIO15 - Acts weird, can be only triggered once (moved to btn C)
   Btn B connected to 32 - Display battery level.
   Btn C connected to 14 - Btn to trigger a measurement.

   NeoPixel LED
   ============

   Adafruit NeoPixel PCB where used (Prod ID: 1612)

   Color codes:
   Cyan: Booting
   Red : Error
   Green: Ready to measure/connect
   White: Measurement busy
   Blue: Bluetooth client connected
   Orange: Battery getting low

   DataPin set to GPIO 12
   
   Analog inputs (FSR inputs)
   ==========================

   A0 - GPIO 26 - uses ADC2 - FSR 1 - Heel Sensor
   A1 - GPIO 25 - uses ADC2 - FSR 2 - 5th Base Metatarsal Sensor 
   A2 - GPIO 34 - uses ADC1 - FSR 3 - 5th Metatarsal Head Sensor
   A3 - GPIO 39 - uses ADC1 - FSR 4 - 1st Metatarsal Head Sensor
   A4 - GPIO 36 - uses ADC1 - FSR 5 - Hallux Sensor  
   A5 - GPIO 04 - Uses ADC2 - NOT CONNECTED.

   Authors
   =======

   Sevit Roy
   
   ADAFRUIT - libs and sample code where used.

   Company
   =======

   MOBILAB - Thomas More Kempen
   
   History
   =======
   2019/Sept/04 - Initial start of this program - RTC support - SD support.
   2019/Sept/05 - Included BT Serial Legacy support to interface with processing application on PC
   2019/Sept/25 - Include support for OLED screen and NEOpixel led
   2019/Sept/26 - Include the ability to start a measurement with button c - Adafruit OLED display button C.
   2019/Ockt/04 - Add battery status indication
   2019/Ockt/07 - Added battery inspection functionality and low battery warning.

*/

// BT Serial librairy
#include "BluetoothSerial.h"

//I2C librairy
#include <Wire.h>

//ADAFRUIT BNO055 driver and sensor object
#include "Adafruit_Sensor_Custom.h"
#include "Adafruit_BNO055_Custom.h"

//Math Lib
#include <math.h>

// RTC lib
#include "RTClib.h"

// SD Card libs
#include "FS.h"
#include "SD.h"
#include <SPI.h>

// OLED Feather Librairies:
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Neopixel librairies
#include <Adafruit_NeoPixel.h>

// FEATHER ECOSYSTEM DEFINES
#define SD_CS 33 // Set to adafruit feather logger SDCS

#define BATT_IN 35 //Battery voltage input on feather board; devided by two so actual value needs to be multiplied.It is located on A13, or GPIO 35

#define OLED_BTN_A 15 // Pushbutton a on OLED feather
#define OLED_BTN_B 32 // Pushbutton a on OLED feather
#define OLED_BTN_C 14 // Pushbutton a on OLED feather

//Trigger to slave
#define SLAVE_TRIGGER 27

//Neopixel pin
#define NEO_PIN 12
#define NEO_NUMPIXELS 1
 
// Global variables

unsigned long previousMillis = 0;
unsigned long elapsedMillis = 0;
unsigned long readingID;

const long intervalPol = 20;

String fileName;
String dataMessage;

int incoming;

bool startLog = false;
bool clientConnected = false;
bool useEuler = false;
bool btnBPressedBefore = false;
bool displayedLowBatStatus = false;

/*  Create Objects/Instances   */

BluetoothSerial SerialBT;
//DFRobot_BNO055 mpu;

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno1 = Adafruit_BNO055(55, BNO055_ADDRESS_A);
Adafruit_BNO055 bno2 = Adafruit_BNO055(55, BNO055_ADDRESS_B);

RTC_PCF8523 rtc; 

Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);

Adafruit_NeoPixel pixels(NEO_NUMPIXELS, NEO_PIN, NEO_GRB + NEO_KHZ800);


void writeClientConnected(){
    display.clearDisplay();
    display.display();
  
    display.setTextSize(1);
    display.setTextColor(WHITE);
    
    display.setCursor(0,0);
    display.print("Client connected");

    display.setCursor(0,10);
    display.print("Send s or press c");
        
    display.display(); // actually display all of the above
}

void writeClientDisconnected(){
    display.clearDisplay();
    display.display();
  
    display.setTextSize(1);
    display.setTextColor(WHITE);
    
    display.setCursor(0,0);
    display.print("Client disconnected");

    display.setCursor(0,10);
    display.print("Connect BT or press c");
        
    display.display(); // actually display all of the above
}


void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param){
  if(event == ESP_SPP_SRV_OPEN_EVT){
    Serial.println("Client Connected");
    clientConnected = true;
    writeClientConnected();
    pixelSetBlue();
  }

  if(event == ESP_SPP_CLOSE_EVT ){
    Serial.println("Client disconnected");  
    clientConnected = false;

    writeClientDisconnected();

    pixelSetGreen();
  }
}

void displaySensorDetailsBNO1(void)
{
  sensor_t sensor;
  bno1.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Details for BNO1"); 
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void displaySensorDetailsBNO2(void)
{
  sensor_t sensor;
  bno2.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Details for BNO2"); 
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

// Write to the SD card (DON'T MODIFY THIS FUNCTION)
void writeFile(fs::FS &fs, const char * path, const char * message) {
//  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if(file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

void pixelSetRed(){
   pixels.clear();
   pixels.setPixelColor(0, pixels.Color(255, 0, 0));
   pixels.show();  
   pixels.setPixelColor(0, pixels.Color(128, 0, 0));
   pixels.show();  
}

void pixelSetOrange(){
   pixels.clear();
   pixels.setPixelColor(0, pixels.Color(255, 165, 0));
   pixels.show();   
}

void pixelSetBlue(){
   pixels.clear();
   pixels.setPixelColor(0, pixels.Color(0, 0, 255));
   pixels.show();   
}

void pixelSetGreen(){
   pixels.clear();
   pixels.setPixelColor(0, pixels.Color(0, 150, 0));
   pixels.show();  
}

void pixelSetCyan(){
   pixels.clear();
   pixels.setPixelColor(0, pixels.Color(0, 255, 255));
   pixels.show();  
}

void pixelSetMagenta(){
   pixels.clear();
   pixels.setPixelColor(0, pixels.Color(255, 0, 255));
   pixels.show();  
}

void pixelSetWhite(){
   pixels.clear();
   pixels.setPixelColor(0, pixels.Color(255, 255, 255));
   pixels.show();  
}


void setup(void)
{

  /* Setup inputs */
  
  pinMode(OLED_BTN_A, INPUT_PULLUP);
  pinMode(OLED_BTN_B, INPUT_PULLUP);
  pinMode(OLED_BTN_C, INPUT_PULLUP);
  
  pinMode(SLAVE_TRIGGER, OUTPUT);
  
  /* SETUP I2C */
  Wire.begin();

  /* SETUP UART */
  Serial.begin(115200);

  /* Setup OLED display */

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  // Clear display buffer

  display.clearDisplay();
  display.display();

  display.setTextSize(1); 
  display.setTextColor(WHITE);

  /* Setup Led Neo Pixel */
  pixels.begin();
  pixelSetCyan();
   
  /* SETUP Real Time Clock */
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");

    
    display.clearDisplay();
    display.display();

    display.setCursor(0,0);
    display.print("RTC Error");

    display.setCursor(0,10);
    display.print("Waiting forever");

    display.display();

    pixelSetRed();
    
    while (1);
  }
  else{
     Serial.println("RTC init succesfully");  

     display.setCursor(0,0);
    
     display.print("RTC init ok.");

     display.display();
  }

  /* INIT BNO1 */
  
  Serial.println("Init of BNO Sensor 1: 0x28"); Serial.println("");

  /* Initialise the sensor */
  if(!bno1.begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS))
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected on adress a... Check your wiring or I2C ADDR!");

    display.clearDisplay();
    display.display();

    display.setCursor(0,0);
    display.print("BNO1 init fail.");

    display.setCursor(0,10);
    display.print("Waiting forever.");

    display.display();  

    pixelSetRed();
     
    while(1);
  }
  else{
     Serial.println("BNO1 init succesfully");  

     display.setCursor(0,10);
    
     display.print("BNO1 init ok.");

     display.display();
  }
   
  delay(1000);

  /* Use external crystal for better accuracy */
  bno1.setExtCrystalUse(true);
   
  /* Display some basic information on this sensor */
  displaySensorDetailsBNO1();

  /* INIT BNO2 */

  Serial.println("Init of BNO Sensor 2: 0x29"); Serial.println("");

  /* Initialise the sensor */
  if(!bno2.begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS))
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected on adress b... Check your wiring or I2C ADDR!");

     display.clearDisplay();
     display.display();

     display.setCursor(0,0);
     display.print("BNO2 init fail.");

     display.setCursor(0,10);
     display.print("Waiting forever.");

     display.display();  

     pixelSetRed();
    
    while(1);
  }
  else{
     Serial.println("BNO2 init succesfully");  

     display.setCursor(0,20);
    
     display.print("BNO2 init ok.");

     display.display();
  }
   
  delay(1000);

  /* Use external crystal for better accuracy */
  bno2.setExtCrystalUse(true);
   
  /* Display some basic information on this sensor */
  displaySensorDetailsBNO2();

  delay(2000);

  display.clearDisplay();
  display.display();



  // The following setup is taken from: http://www.esp32learning.com/code/esp32-and-microsd-card-example.php
      
  if(!SD.begin(SD_CS)) {
     Serial.println("Card Mount Failed");

     display.clearDisplay();
     display.display();

     display.setCursor(0,0);
     display.print("SD mount failed.");

     display.setCursor(0,10);
     display.print("Waiting forever.");

     display.display();  

     pixelSetRed();
     
     while(1);
     return;
  }
  else{
     display.setCursor(0,0);
    
     display.print("SD mount ok.");

     display.display();  
  }
  
  uint8_t cardType = SD.cardType();
      
  if(cardType == CARD_NONE) {
     Serial.println("No SD card attached");
     return;
  }

  Serial.print("SD Card Type: ");
  display.setCursor(0,10);
  display.print("SD type: ");
  display.setCursor(50,10);
  
  if(cardType == CARD_MMC){
    Serial.println("MMC");
    display.print("MMC");
    display.display();
  } else if(cardType == CARD_SD){
    Serial.println("SDSC");
    display.print("SDSC");
    display.display();
  } else if(cardType == CARD_SDHC){
    Serial.println("SDHC");
    display.print("SDHC");
    display.display();
  } else {
    Serial.println("UNKNOWN");
    display.print("UKWN");
    display.display();
  }
  
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  unsigned int cardSized = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);

  display.setCursor(0,20);
  display.print("SD size: ");
  display.setCursor(50,20);
  display.print(cardSized);
  display.display();

  delay(3000);

  display.clearDisplay();
  display.display();

  /* Setup Serial Bluetooth connection */

  Serial.println("ESP32 is Ready to Pair over Bluetooth");

  // Prepare Serial bluetooth lib for a callback function, usefull to see if a client connected or disconnected.

  SerialBT.register_callback(callback);

  if(!SerialBT.begin("ESP32_MOTION")){
    Serial.println("An error occurred initializing Bluetooth");
    
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0,0);
    
    display.print("BT init error!");

    display.display(); // actually display all of the above

    pixelSetOrange();
    
  }else{
    Serial.println("Bluetooth succesfully initialized... Waiting for client connection...");
    
    display.setTextSize(1);
    display.setTextColor(WHITE);
    
    display.setCursor(0,0);
    display.print("BT: ESP32_MOTION.");

    display.setCursor(0,10);
    display.print("Connect client.");

    display.setCursor(0,20);

    display.print("Or press C to start");
        
    display.display(); // actually display all of the above

    pixelSetGreen();
  }

  

  
}

void loop() {
  // put your main code here, to run repeatedly:

  incoming = 0;
  readingID = 0;

  // check if a client connected to the device
  bool previousClientConnectedStatus = clientConnected;
  
  if(clientConnected == true){

    if (SerialBT.available()) //Check if we receive anything from Bluetooth
    {
      incoming = SerialBT.read(); //Read what we recevive
      Serial.print("Received:"); Serial.println(incoming);
      SerialBT.print("I just received: "); SerialBT.print(incoming); SerialBT.println(" over BlueTooth...");
  
    }
  }

  // Check battery status
  
  float batteryLevel = analogRead(BATT_IN);
  batteryLevel = (batteryLevel/4095)*2*3.3*1.1; // multiply by 2 (voltage devider installed by adafruit, 3.3 ESP voltage, 1.1 ADC voltage ref.

  bool btnB_Status = digitalRead(OLED_BTN_B);
  delay(10);
  if (digitalRead(OLED_BTN_B) == true && btnB_Status == false) {
     // B is pressed Display battery voltage

     if (btnBPressedBefore ==  false){
     
       display.clearDisplay();
       display.display();

       display.setTextSize(1);
       display.setTextColor(WHITE);
    
       display.setCursor(0,0);
       display.print("Battery voltage");
       display.setCursor(0,10);
       display.print(batteryLevel);
       display.display(); // actually display all of the above

       btnBPressedBefore = !btnBPressedBefore;
       
     }else{

       display.clearDisplay();
       display.display();
       
       display.setTextSize(1);
       display.setTextColor(WHITE);
    
       display.setCursor(0,0);
       display.print("BT: ESP32_MOTION.");

       display.setCursor(0,10);
       display.print("Connect client.");

       display.setCursor(0,20);

       display.print("Or press C to start");
        
       display.display(); // actually display all of the above

       btnBPressedBefore = !btnBPressedBefore;
     }   
   }

   if (batteryLevel <= 3.5){
       pixelSetOrange();

      if (displayedLowBatStatus==false){

         displayedLowBatStatus = true;
         
         display.clearDisplay();
         display.display();

         display.setTextSize(1);
         display.setTextColor(WHITE);
    
         display.setCursor(0,0);
         display.print("Battery low");
         display.setCursor(0,10);
         display.print("Recharge!");
         display.display(); // actually display all of the above
      }
   }
   

  //Check if button C is pressed
  
  bool btnC_Status = digitalRead(OLED_BTN_C);
  delay(10);
  if (digitalRead(OLED_BTN_C) == true && btnC_Status == false) {
     // C is pressed
     startLog = true;  
  }

  //Serial.print("btn A: ");
  //Serial.println(btnA_Status);
  
  /*Serial.print("startlog: ");
  Serial.println(startLog);

  Serial.print("btn A: ");
  Serial.println(digitalRead(OLED_BTN_A));
  Serial.print("btn B: ");
  Serial.println(digitalRead(OLED_BTN_B));
  Serial.print("btn C: ");
  Serial.println(digitalRead(OLED_BTN_C));*/

  if (incoming == 115 || startLog == true){    // Decimal value (115) for the character s, used here to start a log session, in this stage the code runs for one minute.

    digitalWrite(SLAVE_TRIGGER, HIGH);
    
    startLog = false;

    display.clearDisplay();
    display.display();

    display.setTextSize(1);
    display.setTextColor(WHITE);
    
    display.setCursor(0,0);
    display.print("Measurement started");

    display.setCursor(0,10);
    display.print("Creating log file");
        
    display.display(); // actually display all of the above
    
    Serial.println("start command received");
    SerialBT.print("Start command received, creating a new log file...");

    pixelSetWhite();

    /*========================================================*/
    /* Read RTC value to create a new and unique logging file */
    /*========================================================*/
    
    DateTime rightNow = rtc.now();
  
    String nowYear = String(rightNow.year(), DEC);
  
    String nowMonth = String(rightNow.month(), DEC);
    if (nowMonth.length() == 1){
      nowMonth = String("0"+nowMonth);
    }
  
    String nowDay = String(rightNow.day(), DEC);
    if (nowDay.length() == 1){
      nowDay = String("0"+nowDay);
    }
  
    String nowHour = String(rightNow.hour(), DEC);
    if (nowHour.length() == 1){
      nowHour = String("0"+nowHour);
    }
    
    String nowMinute = String(rightNow.minute(), DEC);
      if (nowMinute.length() == 1){
      nowMinute = String("0"+nowMinute);
    }
    
    String nowSecond = String(rightNow.second(), DEC);
    if (nowSecond.length() == 1){
      nowSecond = String("0"+nowSecond);
    }
    
    fileName = "/" + String(nowYear + nowMonth + nowDay) + "_" + String(nowHour + "H_" + nowMinute + "M_" + nowSecond + "S" + ".txt");
  
    Serial.print("Date and time of this measurement: ");
    Serial.println(fileName);
    SerialBT.print("New log file is named: ");
    SerialBT.println(fileName);

    /*======================================================*/
    /* Create new file on SD card and write the header file */
    /*======================================================*/

    // If the data.txt file doesn't exist
    // Create a file on the SD card and write the data labels
  
    //Modification - Create a file that has the current date and time.
        
    File file = SD.open(fileName);
    
    if(!file) {
      Serial.println("File doens't exist");
      Serial.println("Creating file...");
      //dataMessage = "test";
      //dataMessage = String("Index") + "," + String("Quad W1") + "," + String("Quad X1") + "," + String("Quad Y1") + "," + String("Quad Z1") + "," + String("Quad W2") + "," + String("Quad X2") + "," + String("Quad Y2") + "," + String("Quad Z2") + "," + String("Calculated Angle") + "," + String("Elapsed time (µs)") + "\r\n";
      //dataMessage = String("Index") + "," + String("Quad W1") + "," + String("Quad X1") + "," + String("Quad Y1") + "," + String("Quad Z1") + "," + String("Quad W2") + "," + String("Quad X2") + "," + String("Quad Y2") + "," + String("Quad Z2") + "," + String("Calculated Angle") + "," + String("BNO1 ACC X") + "," + String("BNO1 ACC Y") + "," + String("BNO1 ACC Z") + "," + String("BNO2 ACC X") + "," + String("BNO2 ACC Y") + "," + String("BNO2 ACC Z") + "," + String("BNO1 GYRO X") + "," + String("BNO1 GYRO Y") + "," + String("BNO1 GYRO Z") + "," + String("BNO2 GYRO X") + "," + String("BNO2 GYRO Y") + "," + String("BNO3 GYRO Z") + "," + String("Elapsed time (µs)") + "\r\n";
      
      dataMessage = String("Index") + "," + String("Quad W1") + "," + String("Quad X1") + "," + String("Quad Y1") + "," + String("Quad Z1") + "," + String("Quad W2") + "," + String("Quad X2") + "," + String("Quad Y2") + "," + String("Quad Z2") + "," + String("Calculated Angle") + "," + String("BNO1 ACC X") + "," + String("BNO1 ACC Y") + "," + String("BNO1 ACC Z") + "," + String("BNO2 ACC X") + "," + String("BNO2 ACC Y") + "," + String("BNO2 ACC Z") + "," + String("BNO1 GYRO X") + "," + String("BNO1 GYRO Y") + "," + String("BNO1 GYRO Z") + "," + String("BNO2 GYRO X") + "," + String("BNO2 GYRO Y") + "," + String("BNO3 GYRO Z") + "," + String("ADC 0") + "," + String("ADC 1") + "," + String("ADC 2") + "," + String("ADC 3") + "," + String("ADC 4") + "," +String("Elapsed time (µs)") + "\r\n";
      
      writeFile(SD, fileName.c_str(), dataMessage.c_str());
     
    }
    else {
      Serial.println("File already exists");  
    }
    file.close();    


    /* Open the created file and prepare it to be appended*/
    
    file = SD.open(fileName.c_str(), FILE_APPEND);
  
    if(!file) {
      Serial.println("Failed to open file for appending, halting execution");
      SerialBT.println("Failed to open file for appending, halting execution");
      while(1);
      return;
    }

    Serial.println("File for appending opened... Starting Measurement... ");
    SerialBT.println("File for appending opened... Starting Measurement of 2000 points... ");
    
    unsigned long startTime = millis();
    Serial.println(millis());

    display.setCursor(0,20);
    display.print("Measuring");
    display.display(); // actually display all of the above

    digitalWrite(SLAVE_TRIGGER, LOW);
  
    while(readingID <= 2000){
      
       unsigned long currentMillis = millis();
  
       if (currentMillis - previousMillis >= intervalPol){
        
       // Every 10 mili seconds or 100Hz read bno and write to SD
       previousMillis = currentMillis;
       
       unsigned long startTime = micros(); 
  
       imu::Quaternion quat1 = bno1.getQuat();
       imu::Quaternion quat2 = bno2.getQuat();
  
       imu::Vector<3> acc1 = bno1.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
       imu::Vector<3> acc2 = bno2.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  
       imu::Vector<3> gyro1 = bno1.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
       imu::Vector<3> gyro2 = bno2.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  
       /* Format quaternion values */
       
       String quat1W = String(quat1.w());
       String quat1X = String(quat1.x());
       String quat1Y = String(quat1.y());
       String quat1Z = String(quat1.z());
  
       String quat2W = String(quat2.w());
       String quat2X = String(quat2.x());
       String quat2Y = String(quat2.y());
       String quat2Z = String(quat2.z());
  
       /* Format acc values */
  
       String ACC1X = String(acc1.x());
       String ACC1Y = String(acc1.y());
       String ACC1Z = String(acc1.z());
  
       String ACC2X = String(acc2.x());
       String ACC2Y = String(acc2.y());
       String ACC2Z = String(acc2.z());
  
       /* Format gyro values */
       
       String GYRO1X = String(gyro1.x());
       String GYRO1Y = String(gyro1.y());
       String GYRO1Z = String(gyro1.z());
  
       String GYRO2X = String(gyro2.x());
       String GYRO2Y = String(gyro2.y());
       String GYRO2Z = String(gyro2.z());
  
        // Calculate the dot product:
    
        float dot = quat1.x() * quat2.x()+ quat1.y() * quat2.y() + quat1.z() * quat2.z() + quat1.w() * quat2.w();
        //float dot = quatBNO1.x() * quatBNO2.x()+ quatBNO1.y() * quatBNO2.y() + quatBNO1.z() * quatBNO2.z() + quatBNO1.w() * quatBNO2.w();
        float angle = acos(dot);
        angle = angle * 57.2958 * 2;
    
        // Read and convert ADC values
        int ADC0 = analogRead(36);
        ADC0 = map(ADC0, 0, 4095, 0, 3300);
        int ADC1 = analogRead(39);
        ADC1 = map(ADC1, 0, 4095, 0, 3300);
        int ADC2 = analogRead(34);
        ADC2 = map(ADC2, 0, 4095, 0, 3300);
        int ADC3 = analogRead(35);
        ADC3 = map(ADC3, 0, 4095, 0, 3300);
        int ADC4 = analogRead(15);
        ADC4 = map(ADC4, 0, 4095, 0, 3300);
        ADC4 = ADC4 - 520;
    
        dataMessage = String(readingID) + "," + String(quat1W) + "," + String(quat1X) + "," + String(quat1Y) +"," + String(quat1Z) + "," + String(quat2W) + "," + String(quat2X) + "," + String(quat2Y) +"," + String(quat2Z) + "," + String(angle) + "," + String(ACC1X) + "," + String(ACC1Y) + "," + String(ACC1Z) + "," + String(ACC2X) + "," + String(ACC2Y) + "," + String(ACC2Z) + "," + String(GYRO1X) + "," + String(GYRO1Y) + "," + String(GYRO1Z) + "," + String(GYRO2X) + "," + String(GYRO2Y) + "," + String(GYRO2Z) + "," + String(ADC0) + "," + String(ADC1) + "," + String(ADC2) + "," + String(ADC3) + "," + String(ADC4) + "," + String(elapsedMillis) +"\r\n";
    
        file.print(dataMessage.c_str());
    
        readingID++;
    
        unsigned long endtime = micros();
    
        elapsedMillis = endtime - startTime;
       
  
       }
      
      }
  
      file.close();
      unsigned long endTime = millis();
      Serial.println("Measurement completed");
      Serial.print("Elapsed time: ");
      Serial.println(endTime - startTime); 

      SerialBT.println("Measurement completed and logged to SD.");
      SerialBT.print("Elapsed time: ");
      SerialBT.println(endTime - startTime); 

      display.clearDisplay();
      display.display();

      display.setTextSize(1);
      display.setTextColor(WHITE);
    
      display.setCursor(0,0);
      display.print("Measurement complete");
      display.setCursor(0,10);
      display.print("Press C or connect BT");
      display.display(); // actually display all of the above

      if (clientConnected == true){
         pixelSetBlue();  
      }else{
         pixelSetGreen();  
      }

      
      //while(1);     
    }

}

  
