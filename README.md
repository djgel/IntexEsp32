# Intex® PureSpa ESP32 control Board with Home Automation

**Control your PureSpa over MQTT for 28462, 28458 & 28457, 28561

This  software is user for hardware IntexESP32HW and used with Smart Home (optimized for [Jeedom](www.jeedom.com) & [Home Assistant](https://www.home-assistant.io)).

Codes has partial copy / inspiration of [Yogui79/IntexPureSpa] (https://github.com/Yogui79/IntexPureSpa/blob/main/README.md#hardware-to-you-need)

## Pictures & Videos
-------
- [Video - Home Assistant - Screen Recording ](https://youtu.be/5Z1keongL40)

## Hardware to you need

-   **ESP32 Dev Kit 38Pin**  (Microcontroller) -  [amazon.de](https://amzn.to/3uoa2y4) or [amazon.fr](https://amzn.to/2RvXbLn)]
-   [**Esp32PureSpa**] (Hardware/) Board to make! (in Developed)


## Software to you need

-   [**Arduino IDE**](https://www.arduino.cc/en/software)
-   **Install the “[ESP32 Dev Kit C 38pin](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers)” ** (V1 boards not  work)
-   **Install ESP32 Board for  Arduino IDE (verssion 1.0.6)  ->  [Screenshot](Docs/Image/Board-Screenshot-IDE.PNG). **
-   **Install the following libraries**  (use the Arduino Library Manager)
    -   EspMQTTClient
    -   arduino-timer
    -   WiFi (comming with Board Driver in Arduino IDE)
    -   ESPmDNS (comming with Board Driver in Arduino IDE)
    -   ArduinoOTA
-   **You need a MQTT broker**  (e.g. Mosquitto Broker)

## PureSpa Software
Now you can [Download](src/Spa/) the PureSpa code change it to your settings and upload it to your ESP or Arduino


## MQTT communication protocol

**Write your WIFI settings in this lines**

```
const char* Myssid = "YourSSID";
const char* Mypassword = "YourPassword";
```

----------

**Write your MQTT settings in this lines**

```
"YourMQTT-Broker-IP", 	  // MQTT Broker server ip
"NameMQTTBroker",        // Can be omitted if not needed
"PasswordMQTTBroker",   // Can be omitted if not needed
"IntexSpa",            // Client name that uniquely identif your device. Don't change the name!
 1883                 // The MQTT port, default to 1883. this line can be omitted
```

----------

**MQTT topic & payload**

-   Topic is the path to communtions over MQTT:  
    e.g.:  _**IntexSpa/Cmd Power on off**_
-   Payload is the command 


| Description                  | Topic String                     | Payload | Payload | Only Status           |
|------------------------------|----------------------------------|---------|---------|-----------------|
| **Power**                    | IntexSpa/Cmd Power on off        | ON=1    | OFF=0   | -               |
| **Water Filter**             | IntexSpa/Cmd water filter on off | ON=1    | OFF=0   | -               |
| **Water Filter Timer**        | IntexSpa/Cmd water filter time   | hours=2,4,6       | -   | -               |
| **Bubble**                   | IntexSpa/Cmd bubble on off       | ON=1    | OFF=0   | -               |
| **Heater**                   | IntexSpa/Cmd heater on off       | ON=1    | OFF=0   | -               |
| **Change Farenheit/Celsius** | IntexSpa/Cmd Farenheit Celsius   | F=1     | C=0     | -               |
| **Decrease the Temp.**       | IntexSpa/Cmd decrease            | UP=1    | -       | -               |
| **Increase the Temp.**       | IntexSpa/Cmd increase            | Down=1  | -       | -  
| **ESP Reset**                | IntexSpa/Cmd Reset ESP            | reset | -       | -             |  
| **Command Setpoint of Temp.**       | IntexSpa/Cmd Temperature Setpoint| "set a number"  | -       | -
| **Status Communication with pump**| IntexSpa/Communication with pump | Com/OK=1   | 0=lost connection       | -   |Yes           |
| **Heater Status**                   | IntexSpa/heater state       | standby=1 & ON=2  | OFF=0    | Yes
| **Status Setpoint Temp.**         | IntexSpa/Temperature Setpoint    | -       | -       | Yes
| **Send °F Temp.**     | IntexSpa/Farenheit Celsius       | -       | -       | Yes             |             |
| **Send Actual Temp.**        | IntexSpa/Actual Temperature      | -       | -       | Yes             |
| **Send Error Message**       | IntexSpa/Error Number            | -       | -       | Yes             |
| **Status Power on**          | IntexSpa/Power on                | -       | -       | Yes             |  
| **Status Bubble on**         | IntexSpa/Bubble on               | -       | -       | Yes             |  
| **Status Heater on**         | IntexSpa/heater on               | -       | -       | Yes             |  
| **Status Filter on**         | IntexSpa/filter on               | -       | -       | Yes             |
| **Status Filter Timer**         | IntexSpa/filter setup time               | -       | -       | Yes             |    
| **Water Jet**             |  IntexSpa/Cmd water jet on off | ON=1    | OFF=0   | -               |
| **Sanitizer**               | IntexSpa/Cmd sanitizer on off    | ON=1    | OFF=0   | -               |
| **Status Water Jet**      | IntexSpa/Water jet on          | -       | -       | Yes             |
| **Sanitizer Timer**               | IntexSpa/Cmd Sanitizer time    | hours=3,5,8    | -  | -               |
| **Status Sanitizer**        | IntexSpa/Sanitizer on            | -       | -       | Yes             |
| **Status Sanitizer Timer**        | IntexSpa/Sanitizer setup time            | -       | -       | Yes             |

## Debugging

You can debug on Arduino IDE with serial print on  **baud rate: 115200**

Options to debug  
_Uncomment to debug_  

	//#define DEBUG_RECIEVED_DATA
	//#define DEBUG_SEARCH_CHANNEL
	//#define DEBUG_SEND_COMMAND
	//#define DEBUG_PUMP_DATA
	//#define DEBUG_CONTROLER_DATA
	//#define DEBUG_CONFIG
	//#define DEBUG_MQTT
	//#define DEBUG_SEND_VALUE_TO_HOME_AUTOMATION_SW

## OTA update

You can use OTA update (wireless) via Arduino IDE after the first upload via USB.  [Screenshot](Docs/Image/Screenshot-OTA.PNG)

## Home Assistant
Is a powerful open source home automation software. [www.home-assistant.io](https://www.home-assistant.io)

You can use it as you want, I'll show you an example of a part of the files configuration.yaml and automations.yaml


**Screenshot & Video:**
1. [OFF - Whrilpool Screenshot](Docs/Image/1.HomeAssistant-OFF_Screenshot.jpg) 
2. [ON - Whrilpool Screenshot](Docs/Image/2.HomeAssistant-ON_Screenshot.jpg)
3. [Help/Infos/Reset Screenshot](Docs/Image/3.HomeAssistant-Help-Infos_Screenshot.jpg)
4. [Heat Timer](Docs/Image/4.HomeAssistant-Timer_Screenshot.jpg)
5. [Push-notification Screenshot](Docs/Image/5.HomeAssistant-Push-notification_Screenshot.jpg)
6. [Video - Screen Recording ](https://youtu.be/5Z1keongL40)


**Config of Home Assistant:** 
- [**configuration.yaml**](src/HomeAutomation/HomeAssistant/configuration.yaml)
   - Define the switches and sensors with MQTT Topics and Payload.

- [**automations.yaml**](src/HomeAutomation/HomeAssistant/automations.yaml) **(optional)**

  - All your automation settings such as push-notification on your mobile phone.


## Jeedom 

Is a powerful and innovative open source home automation software. [www.jeedom.com](https://jeedom.com/)

**Screenshot & Video:**
1. [Screenshot dashboard view](Docs/Image/10.Jeedom-Dashboard.png)
2. [Wiring video](https://www.youtube.com/watch?v=50lxF08o_vo) thanks [@Dim](https://github.com/jeedom-help-Dim-Ant)
3. Configuration video **comming soon**.


**Config of Jeedom:** 

Template for the MQTT topics are available inside the beta version from the jeedom plugin jMQTT

**On jeedom Community thread:** 

you can found some information/help (in french) on the [Jeedom Community thread](https://community.jeedom.com/t/intex-purespa-avec-commande-sans-fils/59334)


## The official Intex® PureSpa instructions

| Articel No. | English  | German   |
|-------------|----------|----------|
| 28462, 28458 & 28457, 28561    | [Download](Docs/28462-28458_344IO_202_EN_.pdf) | [Download](Docs/28462_28458_344IO_2020_DE.pdf) |


