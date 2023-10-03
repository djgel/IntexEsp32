/*
 *
 * Intex Purespa Esp32 Board Controler 
 * 
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *

*/

//Now Only  work to Jet&bubbles Delux




/*******************************************************
*
*   S E L E C T  T H E   P R O T O C O L   Y O U   U S E
*  
********************************************************/
//#define _MY_SENSORS_
#define _MQTT_

#ifdef ESP32
const char* Myssid = "MeMs";
const char* Mypassword = "1475963m";
#endif


#if defined (_MY_SENSORS_) &&  defined (_MQTT_)
   #error select between mysensors and MQTT 
#endif

#ifdef _MQTT_
#include "EspMQTTClient.h"

EspMQTTClient client(
  Myssid,
  Mypassword,
  "192.168.31.200",  // MQTT Broker server ip
  "MeMsCasa",   // Can be omitted if not needed
  "",   // Can be omitted if not needed
  "IntexSpa",     // Client name that uniquely identify your device
  1883              // The MQTT port, default to 1883. this line can be omitted
);
#endif

//Uncomment following line to have more debug infos
//#define DEBUG_RECIEVED_DATA
//#define DEBUG_SEARCH_CHANNEL
//#define DEBUG_SEND_COMMAND
//#define DEBUG_PUMP_DATA
//#define DEBUG_CONTROLLER_DATA
//#define DEBUG_CONFIG
//#define DEBUG_MQTT
//#define DEBUG_SEND_VALUE_TO_HOME_AUTOMATION_SW

bool FirstSend;

#include <arduino-timer.h>

#define mySerial   Serial2
#define DO_SET    19
#define DO_CS     18



// PINs
#define PUMP_PIN       16
#define BUBBLES_PIN    17
#define JET_PIN        5
#define HEATER01_PIN   18
#define HEATER02_PIN   19
#define CL01_PIN       27
#define CL02_PIN       14
#define ECO_PIN        4
#define TEMP01_PIN     35
#define TEMP02_PIN     34
#define FLOW01_PIN     22
#define FLOW02_PIN     23
#define TFUSE_PIN      32
#define SDA_PIN        25
#define SCL_PIN        26


#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <Adafruit_INA219.h>


//Amp Modulo INA219

Adafruit_INA219 ina219_0 (0x40);
Adafruit_INA219 ina219_1 (0x41);

//Version and date
#define _VERSION         "1.0"
#define _BUILD_DATE_TIME "2023.10.03 "


//Command
#define COMMAND_ON_OFF                0x0001    
#define COMMAND_WATER_FILTER          0x0010    
#define COMMAND_BUBBLE                0x0020    
#define COMMAND_HEATER                0x0040    
#define COMMAND_CELSIUS_FARENHEIT     0x0002    
#define COMMAND_DECREASE              0x0008    
#define COMMAND_INCREASE              0x0004    
#define COMMAND_WATER_JET             0x0100    
#define COMMAND_SANITIZER             0x0080    

// Status Byte number
#define BYTE_STATUS_STATUS            2
#define BYTE_STATUS_COMMAND           4

#define BYTE_ACTUAL_TEMPERATURE       5
#define BYTE_SETPOINT_TEMPERATURE     7

#define BYTE_SETPOINT_TIME_SANITIZER  8

#define BYTE_SETPOINT_TIME_FILTER     12

#define BYTE_ERROR                    14

//Value Pump bytes
#define VALUE_CONTROLLER_ON           0x01
#define VALUE_BUBBLE_ON               0x10

#define VALUE_HEATER_STANDBY          0x02
#define VALUE_HEATER_ON               0x04  
#define VALUE_WATER_FILTER_ON         0x08  

#define VALUE_WATER_JET_ON            0x80  
#define VALUE_SANITIZER_ON            0x20  


#define VALUE_FARENHEIT               0x01
#define VALUE_TEMP_WINDOWS_OPEN       0x02
#define VALUE_COMMAND_RECIVED         0x80

// Commande bytes
#define BYTE_CONTROLLER_LOADING        3

// Value Controller bytes

//varibles for datamanagement
#define SIZE_CONTROLLER_DATA         8
unsigned char DataController[SIZE_CONTROLLER_DATA +2];

//Id for mysensors and for memo
uint16_t MemValueSended[40];

#define ID_POWER_ON                 1
#define ID_BUBBLE_ON                2
#define ID_HEATER_ON                3
#define ID_HEATER_STATE             4
#define ID_WATER_FILTER_ON          5
#define ID_WATER_FILTER_TIME        6
#define ID_VALUE_WATER_JET_ON       7
#define ID_SANITIZER_ON             8
#define ID_SANITIZER_TIME           9

#define ID_FARENHEIT                15

#define ID_SETPOINT_TEMPERATURE     20
#define ID_ACTUAL_TEMPERATURE       21
#define ID_TARGET_TEMPERATURE       22
#define ID_TARGET_FILTER_TIME       23
#define ID_TARGET_SANITIZER_TIME    24

#define ID_ERROR_CODE               35
#define ID_COM_PUMP                 36

#define SIZE_PUMP_DATA               17
unsigned char Data[SIZE_PUMP_DATA +2];
uint16_t DataCounter=0;

//internal variables
auto t = timer_create_default(); // create a timer with default settings
bool FinishPumpMessage;
bool FinishControllerMessage;
char ControllerLoadingState;
uint16_t state;
uint16_t CommandToSend =0x0000;
bool CommandRecived;
long LastTimeReciveData;
bool ErrorCommunicationWithPump;
long LastTimeSendData;
long LastTimeReciveDataCheckChannel;
uint8_t ActualSearchChannel;
uint16_t SearchChannelDataCount; 
uint8_t UsedChannel;
uint8_t FirstCommandChar;
uint16_t ChannelChangeOk;
bool FarenheitCelsius;
uint8_t ActualSetpointTemperarue;
uint8_t TargetSetpointTemperarue;
bool ChangeTargetSetpointTemperarue;
bool ChangeSetpointRecirculationTime;
uint8_t ActualSetpointRecirculationTime;
uint8_t TargetSetpointRecirculationTime;
bool SwitchOffRecirculation;
bool StateRecirculation;
bool ChangeSetpointSanitizerTime;
uint8_t ActualSetpointSanitizerTime;
uint8_t TargetSetpointSanitizerTime;
bool SwitchOffSanitizer;
bool StateSanitizer;


//Setup
void setup() {
 
  mySerial.begin(9600);
  Serial.begin(115200);

  Serial.println(F("----------------------------------------------------------------------------------------------------------------------")); 
  Serial.println(F(""));
  Serial.print (F("          Intex Purespa communication sketch version "));  
  Serial.print (_VERSION);
  Serial.print (F(" build on : "));
  Serial.println (_BUILD_DATE_TIME);  
  Serial.println(F(""));
  Serial.println(F("----------------------------------------------------------------------------------------------------------------------"));  
  

  Serial.println("Booting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(Myssid, Mypassword);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  

  
#if  defined (_MQTT_) &&  defined (DEBUG_MQTT)
  client.enableDebuggingMessages(); // Enable debugging messages sent to serial output
#endif  

  delay(1000);
  
  digitalWrite(DO_CS, LOW);     
  digitalWrite(DO_SET, LOW);
  delay(100);

  //read channel inside EERPOM 
  t.every(1600, SendLifeFct);
  LastTimeReciveData = LastTimeSendData= millis();
}// void setup()

void loop() {
   //update timer
   t.tick();


  ArduinoOTA.handle();

   
   if (!FinishPumpMessage)
   {
    {
       if (mySerial.available() ) 
       {
        unsigned char c = mySerial.read();
        ReadData(c);
       }
     }
   }

   //Manage pump message
   if (FinishPumpMessage)
   {
     LastTimeReciveData = millis();
 #ifdef _MQTT_ 
     if (client.isConnected())
 #endif
     {
       DataManagement();
       SendCommandManagement (&CommandToSend);
       SendTemperatureSetpoint();
       SendSpecialCommand(COMMAND_WATER_FILTER, &ChangeSetpointRecirculationTime, SwitchOffRecirculation,TargetSetpointRecirculationTime, ActualSetpointRecirculationTime);
       SendSpecialCommand(COMMAND_SANITIZER, &ChangeSetpointSanitizerTime, SwitchOffSanitizer,TargetSetpointSanitizerTime, ActualSetpointSanitizerTime);
       state =0;
       FirstSend = true;
       FinishPumpMessage = false;  
     }
     
   }
  
   //Manage Controller message
   if (FinishControllerMessage)
   {
#ifdef DEBUG_CONTROLLER_DATA
    Serial.print("Debug Controller data : ");
    char res2[5]; 
    for (uint8_t i =0; i<SIZE_CONTROLLER_DATA;i++){
      sprintf(&res2[0],"%02X",DataController[i]);
      Serial.print(res2);
      Serial.print(" ");
    }
    Serial.println(" ");
#endif    
      ControllerLoadingState = DataController[BYTE_CONTROLLER_LOADING];
      FinishControllerMessage = false;
   }

  client.loop();
  if (   !client.isConnected()
      && !FirstSend 
     ){
      LastTimeReciveData = millis();
     }

  // check communication pump timeout

    {  
      if (millis() - LastTimeReciveData > 5000)
        {
        SendValue("IntexSpa/Communication with pump", false,ID_COM_PUMP); 
        ErrorCommunicationWithPump = true;     
      }
      else{
        SendValue("IntexSpa/Communication with pump", true,ID_COM_PUMP); 
        ErrorCommunicationWithPump = false;
      }
    }
  //Try to find the new channel in case of pump change channel
  if (   !FirstSend 
      &&  (millis() - LastTimeReciveData > 6000 )

      &&  client.isConnected()      

     ){
     if (ChannelChangeOk == 1){ 
        ActualSearchChannel =UsedChannel+1;
     }

     else{
        ActualSearchChannel =UsedChannel -10;
     }
     UsedChannel=0;
     ChannelChangeOk = 0;
     Serial.println (F("Look like channel change since last boot"));
     client.publish("IntexSpa/Notification", "Search the device"); 
     while (!UsedChannel &&  (millis() - LastTimeReciveData < 420000 ))
       SearchChannel();
     if (UsedChannel)
      client.publish("IntexSpa/Notification", "Device found"); 
    }

} //void loop() 

//Managed data recived from the LC12s
void ReadData (unsigned char c)
{ 
   switch (state) 
   {
      case 0: //wait header
      {
        DataCounter=0;
        memset (&Data,0,sizeof(Data));
        memset (&DataController,0,sizeof(DataController));
        
        if (c == FirstCommandChar)
        {
          Data[DataCounter] =c;
          DataCounter++;
          state++;
        }
        break;
      }
      //wait all other data
      case 1:
      {
        Data[DataCounter] =c;
        DataCounter++;

        //it was a message from controller
        if (c == FirstCommandChar && DataCounter< SIZE_PUMP_DATA  && DataCounter > SIZE_CONTROLLER_DATA )
        {     
          memcpy (&DataController,&Data, 8);          
          //Calclulate an check Checksum
          uint16_t crc_out = calc_crc((char*) DataController,SIZE_CONTROLLER_DATA -2);      
          //Checksum ok
          if (   (Data[SIZE_CONTROLLER_DATA-2] == ((crc_out & 0xFF00) >> 8 ))
              && (Data[SIZE_CONTROLLER_DATA-1] == (crc_out & 0x00FF))
             )    
          {
            FinishControllerMessage = true;
          }          
          else
          {
            FinishControllerMessage = false;
          }
         
          DataCounter = 0;
          Data[DataCounter] =c;
          DataCounter++;
        }
        // wait until full pump message is recived
        if (DataCounter> SIZE_PUMP_DATA-1)
        {
          //Calclulate an check Checksum
          uint16_t crc_out = calc_crc((char*)Data,SIZE_PUMP_DATA -2);      
          //Checksum ok
          if (   (Data[SIZE_PUMP_DATA-2] == ((crc_out & 0xFF00) >> 8 ))
              && (Data[SIZE_PUMP_DATA-1] == (crc_out & 0x00FF))
             )    
          {
            state =0;
            FinishPumpMessage = true;
          }
          else{ // Wrong Cheksum
            state =0;
          }         
        }
        break;
      }
   }
}  

//manage information comming from the home automation sofware over MQTT
#ifdef _MQTT_
void onConnectionEstablished()
{
   //manage command power on/off
    client.subscribe("IntexSpa/Cmd Power on off", [](const String & payload) {
      if (payload== "1" || payload== "0"){
        CommandToSend |= COMMAND_ON_OFF ;
        LastTimeSendData = millis();
      }
    });

    //Target Filter time
    client.subscribe("IntexSpa/Cmd water filter time", [](const String & payload) {
      if( (payload== "2" || payload == "4" || payload == "6") ){
        ChangeSetpointRecirculationTime =  StateRecirculation;
        TargetSetpointRecirculationTime = payload.toInt();
             
      }
    });

   //manage command water filter on/off
   client.subscribe("IntexSpa/Cmd water filter on off", [](const String & payload) { 
   if (payload== "1" && (TargetSetpointRecirculationTime== 2 || TargetSetpointRecirculationTime == 4 || TargetSetpointRecirculationTime == 6)){
      ChangeSetpointRecirculationTime = true;
      SwitchOffRecirculation = false;
    }
    else if(payload== "0"){
      ChangeSetpointRecirculationTime = true;
      SwitchOffRecirculation = true;
    }
  });

   //manage command Bubble on/off
   client.subscribe("IntexSpa/Cmd bubble on off", [](const String & payload) {
    if (payload== "1" || payload== "0"){
      CommandToSend |= COMMAND_BUBBLE;
      LastTimeSendData = millis();
    }
  });

   //manage command heater on/off
   client.subscribe("IntexSpa/Cmd heater on off", [](const String & payload) {
    if (payload== "1" || payload== "0"){
    CommandToSend |= COMMAND_HEATER;
      LastTimeSendData = millis();
    }    
  });  

   //manage command Farenheit/Celsius
   client.subscribe("IntexSpa/Cmd Farenheit Celsius", [](const String & payload) {
    if (payload== "1" || payload== "0"){
      CommandToSend |= COMMAND_CELSIUS_FARENHEIT;
      LastTimeSendData = millis();
    }      
  }); 
    
   //manage command decrease
   client.subscribe("IntexSpa/Cmd decrease", [](const String & payload) {
    if (payload== "1")
   {
      LastTimeSendData = millis();
      TargetSetpointTemperarue = TargetSetpointTemperarue -1;
      ChangeTargetSetpointTemperarue = true;     
    }
  }); 

   //manage command increase
   client.subscribe("IntexSpa/Cmd increase", [](const String & payload) {
    if (payload== "1"){
      LastTimeSendData = millis();
      TargetSetpointTemperarue = TargetSetpointTemperarue +1;
      ChangeTargetSetpointTemperarue = true;
    }
  });   

   //manage command temperature setpoint
   client.subscribe("IntexSpa/Cmd Temperature Setpoint", [](const String & payload) {
    TargetSetpointTemperarue = payload.toInt();
    ChangeTargetSetpointTemperarue = true;
  }); 

   // reset
   client.subscribe("IntexSpa/Cmd Reset ESP", [](const String & payload) {
    if (payload== "reset"){
      ESP.restart();
    }
  });   


   //manage command water jet on off
   client.subscribe("IntexSpa/Cmd water jet on off", [](const String & payload) {
    if (payload== "1" || payload== "0"){
      CommandToSend |= COMMAND_WATER_JET;
      LastTimeSendData = millis();
    }
   });   

    //Target Sanitizer time
   client.subscribe("IntexSpa/Cmd Sanitizer time", [](const String & payload) {
      if( (payload== "3" || payload == "5" || payload == "8") ){
        ChangeSetpointSanitizerTime =  StateSanitizer;
        TargetSetpointSanitizerTime = payload.toInt();            
      }
    });

   //manage command sanitizer on/off
   client.subscribe("IntexSpa/Cmd sanitizer on off", [](const String & payload) { 
   if (payload== "1" && (TargetSetpointSanitizerTime== 3 || TargetSetpointSanitizerTime == 5 || TargetSetpointSanitizerTime == 8)){
      ChangeSetpointSanitizerTime = true;
      SwitchOffSanitizer = false;
    }
    else if(payload== "0"){
      ChangeSetpointSanitizerTime = true;
      SwitchOffSanitizer = true;
    }
  });


  
}
#endif

// manage the data recived from the pump and send it to the home autmation software over MQTT
void DataManagement (){
#ifdef DEBUG_PUMP_DATA
    char res[5]; 
    Serial.print("Debug Pump data : ");    
    for (uint8_t i =0; i<SIZE_PUMP_DATA;i++){
      sprintf(&res[0],"%02X",Data[i]);
      Serial.print(res);
      Serial.print(" ");
    }
    Serial.println(" ");
#endif

   // Send power on
   SendValue("IntexSpa/Power on", (bool)(Data[BYTE_STATUS_COMMAND] & VALUE_CONTROLLER_ON),ID_POWER_ON); 

   //Send Bubble on 
   SendValue("IntexSpa/Bubble on", (bool)(Data[BYTE_STATUS_COMMAND] & VALUE_BUBBLE_ON),ID_BUBBLE_ON); 

   //Send heater on 
   SendValue("IntexSpa/heater on", (bool)((Data[BYTE_STATUS_COMMAND] & VALUE_HEATER_ON) || (Data[BYTE_STATUS_COMMAND] & VALUE_HEATER_STANDBY)),ID_HEATER_ON); 

   //Send heater State
   uint16_t HeaterState= 0;
   if (Data[BYTE_STATUS_COMMAND] & VALUE_HEATER_STANDBY){
      HeaterState = 1;      // state standby
   }else if ((Data[BYTE_STATUS_COMMAND] & VALUE_HEATER_ON) ){
     HeaterState = 2;       // state heater on
   }
   SendValue("IntexSpa/heater state", HeaterState ,ID_HEATER_STATE); 

   //Send water filter on 
   StateRecirculation =(bool)(Data[BYTE_STATUS_COMMAND] & VALUE_WATER_FILTER_ON);
   SendValue("IntexSpa/filter on", (bool)(Data[BYTE_STATUS_COMMAND] & VALUE_WATER_FILTER_ON),ID_WATER_FILTER_ON); 

   //Send actual filter setup time
   ActualSetpointRecirculationTime = Data[BYTE_SETPOINT_TIME_FILTER];
   SendValue("IntexSpa/filter setup time", Data[BYTE_SETPOINT_TIME_FILTER],ID_WATER_FILTER_TIME); 
   if (!ChangeSetpointRecirculationTime && ActualSetpointRecirculationTime)
   {
       TargetSetpointRecirculationTime =ActualSetpointRecirculationTime;
       SendValue("IntexSpa/Cmd water filter time", Data[BYTE_SETPOINT_TIME_FILTER],ID_TARGET_FILTER_TIME);   
   }

   //Send water jet on 
   SendValue("IntexSpa/Water jet on", (bool)(Data[BYTE_STATUS_COMMAND] & VALUE_WATER_JET_ON),ID_VALUE_WATER_JET_ON); 

  //Send water sanitizer on 
   StateSanitizer =(bool)(Data[BYTE_STATUS_COMMAND] & VALUE_SANITIZER_ON);
   SendValue("IntexSpa/Sanitizer on", (bool)(Data[BYTE_STATUS_COMMAND] & VALUE_SANITIZER_ON),ID_SANITIZER_ON); 

   //Send actual filter setup time
   ActualSetpointSanitizerTime = Data[BYTE_SETPOINT_TIME_SANITIZER];
   SendValue("IntexSpa/Sanitizer setup time", Data[BYTE_SETPOINT_TIME_SANITIZER],ID_SANITIZER_TIME); 
   if (!ChangeSetpointSanitizerTime && ActualSetpointSanitizerTime)
   {
       TargetSetpointSanitizerTime =ActualSetpointSanitizerTime;
       SendValue("IntexSpa/Cmd Sanitizer time", Data[BYTE_SETPOINT_TIME_SANITIZER],ID_TARGET_SANITIZER_TIME);   
   }  
  
   
   //Send Farenheit selected
   FarenheitCelsius = (bool)(Data[BYTE_STATUS_STATUS] & VALUE_FARENHEIT);
   SendValue("IntexSpa/Farenheit Celsius", (bool)(Data[BYTE_STATUS_STATUS] & VALUE_FARENHEIT),ID_FARENHEIT); 

   //Send temperature setpoint
   ActualSetpointTemperarue = Data[BYTE_SETPOINT_TEMPERATURE];
   SendValue("IntexSpa/Temperature Setpoint", Data[BYTE_SETPOINT_TEMPERATURE],ID_SETPOINT_TEMPERATURE); 
   if (!ChangeTargetSetpointTemperarue)
   {
       TargetSetpointTemperarue =ActualSetpointTemperarue;
       SendValue("IntexSpa/Cmd Temperature Setpoint", Data[BYTE_SETPOINT_TEMPERATURE],ID_TARGET_TEMPERATURE);   
   }

   //Send actual temperature
   SendValue("IntexSpa/Actual Temperature", Data[BYTE_ACTUAL_TEMPERATURE],ID_ACTUAL_TEMPERATURE); 

  //send Error number
   SendValue("IntexSpa/Error Number", Data[BYTE_ERROR],ID_ERROR_CODE); 

  //Manage Command recived
  if (Data[BYTE_STATUS_STATUS] & VALUE_COMMAND_RECIVED){
    CommandRecived = true;
  }
  else{
    CommandRecived = false;
  }
}

//manage send temperature to pump
void SendTemperatureSetpoint(){
  
  //nothing to do 
  if (!ChangeTargetSetpointTemperarue)
    return;
    
  // command is pendling actualy nothing else to do   
  if (    (CommandToSend & COMMAND_INCREASE)
      ||  (CommandToSend & COMMAND_DECREASE)
     )
    return;
    
  // setpoint is done  
  if (    TargetSetpointTemperarue == ActualSetpointTemperarue
      ||  (TargetSetpointTemperarue > 40  && !FarenheitCelsius)
      ||  (TargetSetpointTemperarue < 10  && !FarenheitCelsius)
      ||  (TargetSetpointTemperarue > 50  &&  FarenheitCelsius)
      ||  (TargetSetpointTemperarue < 104 &&  FarenheitCelsius)
     ){
    ChangeTargetSetpointTemperarue   = false;
    return;
  }
    
  //temperature need to be increased -> send increase command
  if (TargetSetpointTemperarue > ActualSetpointTemperarue)
  {
      LastTimeSendData = millis();
      CommandToSend |= COMMAND_INCREASE;
  }

  //temperature need to be decreased -> send decrease command
  if (TargetSetpointTemperarue < ActualSetpointTemperarue)
  {
      LastTimeSendData = millis();
      CommandToSend |= COMMAND_DECREASE;
  }
}

//function to send Recirculation and sanizer command 
void SendSpecialCommand( uint16_t Command, bool *CommandChange, bool CommandOff,uint8_t SetupValue ,uint8_t ActualValue ){

    if (     !Command 
          || !*CommandChange 
       )
    return;

  //nothing to do 
  if (!CommandChange)
  {
    CommandOff =false;
    return;
  }
    
  // command is pendling actualy nothing else to do   
  if (CommandToSend & Command)
  {
    return;
  }

  // Sepoint is done
  if (    ((SetupValue == ActualValue)&& !CommandOff)
      ||  ((ActualValue == 0x00) && CommandOff)
     )
  {
    *CommandChange   = false;
    return;
  }
  // send command
  LastTimeSendData = millis();
  CommandToSend |= Command;
  
}

#ifdef _MY_SENSORS_
void MySensorsCommandManagement(){


}

#endif

//manage the command to the pump;  send until it recived
void SendCommandManagement (uint16_t *Command){
  
  if (!Command || !*Command)
    return;

  if(    CommandRecived
     ||  (millis() -LastTimeSendData> 2000) // timeout
    )
  {
    *Command = 0x0000;
    return;
  }
  SendCommand(*Command);

}

//Send commant to recive all info in case the controller is not close to pump (E81 on controller or off)
bool SendLifeFct(void *)
{
#ifdef _MQTT_ 
    if (client.isConnected() && !ErrorCommunicationWithPump)
#endif
    {
      if (!CommandToSend)
        SendCommand(0x0000);
    }
    return true;
}

//Send command to pump
void SendCommand(uint16_t Command){
  // Sample :C7 00 00 80 00 00 4D 2B
  unsigned char SendData[10];
#ifdef DEBUG_SEND_COMMAND 
  char res[5];
#endif  
  SendData[0] = FirstCommandChar;
  SendData[1] = (Command & 0xFF00) >> 8;;
  SendData[2] = Command  & 0x00FF;
  SendData[3] = ControllerLoadingState;  
  SendData[4] = 0x00;
  SendData[5] = 0x00;

  //checksum
  uint16_t crc_out = calc_crc((char*) SendData,6);
  SendData[6] = (crc_out & 0xFF00) >> 8;
  SendData[7] = crc_out & 0x00FFF;
#ifdef DEBUG_SEND_COMMAND 
  Serial.print("Debug Send command : ");
#endif 
  for (uint8_t i =0; i<SIZE_CONTROLLER_DATA;i++){
#ifdef DEBUG_SEND_COMMAND 
    sprintf(&res[0],"%02X",SendData[i]);
    Serial.print(res);
    Serial.print(" ");
#endif 
    //Wreite commans to LC12s Serial
    mySerial.write(SendData[i]);
  }
#ifdef DEBUG_SEND_COMMAND   
    Serial.println(" ");
#endif    
}


//---------------------------------------------------
//for sending values
// send uint8_t value only if changed
void SendValue(const String &topic, uint8_t value ,int SENSOR_ID){
  if (value != (uint8_t)MemValueSended[SENSOR_ID]||(!FirstSend && !ErrorCommunicationWithPump))
  {
#ifdef _MY_SENSORS_
 //   send( msg.setSensor( SENSOR_ID ).set( value) );
#endif    
#ifdef _MQTT_
    if (client.isConnected())
      client.publish(topic, String(value));     
#endif
        
#ifdef DEBUG_SEND_VALUE_TO_HOME_AUTOMATION_SW    
    // Print to value consol
    Serial.print(topic);
    Serial.print(" ");
    Serial.println (value);
#endif    
#ifdef _MQTT_
    if (client.isConnected())
#endif
    {
      MemValueSended[SENSOR_ID] = value;
    }
  }   
  
}