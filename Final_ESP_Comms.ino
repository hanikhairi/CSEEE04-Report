/***************************************************************
 * $ Bioreactor :: CSEEE Group4 $
 *
 * Title: Final_ESP_Comms.ino
 * Author: Uzair Khan (uzair.khan.21@ucl.ac.uk)
 * Author: Satbir Virdi (satbir.virdi.21@ucl.ac.uk)
 * Date: 2021/12/11
 * Version: 1.0
 **************************************************************/

//The RPC_Response functions are heavily adpated from:
//https://moodle.ucl.ac.uk/pluginfile.php/4381816/mod_resource/content/4/Bioreactor%20connectivity%20overview.pdf

//The functions (sendSlave, requestData) for communication between ESP32 and Arduino is adapted from:
//https://moodle.ucl.ac.uk/pluginfile.php/4381816/mod_resource/content/4/Bioreactor%20connectivity%20overview.pdf

//The logic behind sending the float array is adapted from the first answer from:
//https://arduino.stackexchange.com/questions/63586/is-it-possible-to-send-a-float-array-over-i2c 


#include <Wire.h>
// Define Slave I2C Address. All slaves must be allocated a // unique number.
#define SLAVE_ADDR 9
// Define the pins used for SDA and SCL. This is important because // there is a problem with the TTGO and I2C will not work properly // unless you do.
#define I2C_SDA 21
#define I2C_SCL 22

//All libraries which are used
#include <WiFi.h> //Wifi
#include <ThingsBoard.h>// WiFi control for ESP32
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
  // WiFi control for ESP32
  
  
#define COUNT_OF(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))

// See https://thingsboard.io/docs/getting-started-guides/helloworld/
#define TOKEN               "g13KlR8EJfzb40J618Lh"
#define THINGSBOARD_SERVER  "demo.thingsboard.io"

const char* ssid = "<insert SSID>";
const char* password = "<insert Password>";

int count = 1;
void sendSlave(float num)
{
  float myData[1];
  myData[0] = num;
  Wire.beginTransmission(SLAVE_ADDR);
  Wire.write((uint8_t*) myData, sizeof(myData));
  Wire.endTransmission();
}

int quant = 1;                  // Main application loop delay
float FloatData = 100.00;// Initial update delay.
int lastUpdate  = 0;            // Time of last update.
bool subscribed = false;        // Set to true if application is subscribed for the RPC messages

WiFiClient espClient;     //initialise thingsboard
ThingsBoard tb(espClient);
PubSubClient client(espClient);
int status = WL_IDLE_STATUS;    //Wifi radio status
static uint16_t messageCounter = 0; // count values sent

int RPMdata = 500;


void reconnect() {
  // Loop until we're reconnected
  status = WiFi.status();
  if ( status != WL_CONNECTED) {
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("Reconnected to WiFi");
  }
}


RPC_Response processSetRPM(const RPC_Data &data)
{
  Serial.println("Received the set delay RPC method RPM");
  
  sendSlave(data);  //send target value to Arduino
  Serial.print("Set new RPM: ");
  Serial.println(data);
  return RPC_Response(NULL, RPMdata);
}

RPC_Response processGetRPM(const RPC_Data &data)
{
  Serial.println("Received the RPM"); //confirms RPM received
  return RPC_Response(NULL, RPMdata);
}

//The get/set methods are repeated for pH and Temperature
RPC_Response processSetpH(const RPC_Data &data)
{
  Serial.println("Received the set delay RPC method pH");
  sendSlave(data);
  Serial.print("Set new pH: ");
  Serial.println(data);

  return RPC_Response(NULL, data);
}

RPC_Response processGetpH(const RPC_Data &data)
{
  Serial.println("Received the pH:");

  return RPC_Response(NULL, data);
}

RPC_Response processSetTemp(const RPC_Data &data)
{
  Serial.println("Received the set delay RPC method temp");
  
 
  sendSlave(data);
  updateDelay = data;
  Serial.print("Set new Temp: ");
  Serial.println(data);
  return RPC_Response(NULL, data);
}

RPC_Response processGetTemp(const RPC_Data &data)
{
  Serial.println("Received the temperature");
  return RPC_Response(NULL, data);
}

//RPC_Callback is heavily adpated from:
//https://moodle.ucl.ac.uk/pluginfile.php/4381816/mod_resource/content/4/Bioreactor%20connectivity%20overview.pdf

RPC_Callback callbacks[] = {
  { "getTemp",         processGetTemp },
  { "setTemp",         processSetTemp },
  { "getpH",         processGetpH},
  { "setpH",         processSetpH },
  { "getRPM",         processGetRPM },
  { "setRPM",         processSetRPM },
};


void setup() 
{
  Wire.begin (I2C_SDA, I2C_SCL); 
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
}

//Requests data in the form of a float array containing three values, in the order of temperature reading, RPM reading and pH reading.
void requestData(uint8_t* pointer)
{
  float sizearray[3];
  Wire.requestFrom(SLAVE_ADDR, sizeof(sizearray));
  while (Wire.available()) 
  {
    *pointer++ = Wire.read();
  }
}



void loop() { 
  
  float tempVal;
  float rpmVal;
  float phVal;
  float currentParameters[3];
  uint8_t* parameterPointer = (uint8_t*) currentParameters;
  
  requestData(parameterPointer);

  tempVal =currentParameters[0];
  rpmVal = currentParameters[1];
  phVal = currentParameters[2];
  Serial.println("Received Temperature (Simulated data as subsystem not sending values):");
  Serial.println(tempVal);
  Serial.println("Received RPM:");
  Serial.println(rpmVal);
  Serial.println("Received pH (Simulated data as subsystem not sending values):");
  Serial.println(phVal);

//sanity checking using ranges and error margins

  if(tempVal < 24.5 || tempVal > 35.5)
  {
    Serial.println("THE TEMPERATURE IS OUT OF RANGE");
   
    uint8_t* parameterPointer = (uint8_t*) currentParameters;
  
    requestData(parameterPointer);

    tempVal =currentParameters[0];
    rpmVal = currentParameters[1];
    phVal = currentParameters[2];
  }
  if(rpmVal < 480 || rpmVal > 1520)
  {
    uint8_t* parameterPointer = (uint8_t*) currentParameters;
  
    requestData(parameterPointer);

    tempVal =currentParameters[0];
    rpmVal = currentParameters[1];
    phVal = currentParameters[2];

  }
  if(phVal < 2.9|| phVal > 7.1)
  {
    Serial.println("THE PH IS OUT OF RANGE");
    uint8_t* parameterPointer = (uint8_t*) currentParameters;
  
    requestData(parameterPointer);

    tempVal =currentParameters[0];
    rpmVal = currentParameters[1];
    phVal = currentParameters[2];
    
  }
 
  

  


  delay(500);

  
  delay(1000);
  // Reconnect to WiFi, if needed
  if (WiFi.status() != WL_CONNECTED) {
    reconnect();
    return; 
    }
  // Reconnect to ThingsBoard, if needed
  // The thingsboard connection and the RPC subscription is heavily adapted from:
  ///https://moodle.ucl.ac.uk/pluginfile.php/4381816/mod_resource/content/4/Bioreactor%20connectivity%20overview.pdf
  if (!tb.connected()) {
    subscribed = false;
  // Connect to the ThingsBoard 
    Serial.print("Connecting to: "); 
    Serial.print(THINGSBOARD_SERVER); 
    Serial.print(" with token "); 
    Serial.println(TOKEN);
    if (!tb.connect(THINGSBOARD_SERVER, TOKEN)) {
      Serial.println("Failed to connect");
      return;
      } 
  }
  if (!subscribed) {
    Serial.println("Subscribing for RPC...");

    // Perform a subscription. All consequent data processing will happen in
    // callbacks as shown by callbacks[] array.
    if (!tb.RPC_Subscribe(callbacks, COUNT_OF(callbacks))) {
      Serial.println("Failed to subscribe for RPC");
      return;
    }
     Serial.println("Subscribe done");
     subscribed = true;
  }//send telemetry with the aliases as shown in the quotation marks 
     tb.sendTelemetryFloat("Temperature", tempVal);
     tb.sendTelemetryFloat("RPM", rpmVal);
     tb.sendTelemetryFloat("pH", phVal);

  tb.loop();
}
 
  
