/***********************************************************************************
 * SMAA V1.0 Air Quality monitor | smability.io
 * Author: HJ
 * Date: 02/24
 * Version:1.0
 * Latest update :02/27
 * ************************************************************************************/
 /*
 * Notes 02/24 (mark when done)
 *  1. Include Device Mode (0) o (1) (Done)
 *  2. URL Parse of format lat, long -->string(lat,long)=gpsData; send GPS data as a string in one varible (Done)
 *  2a.URL parsing all variables, use dummy variables when need it -19-02-24- (Done)
 *  3. Set GPS Enable pin-(Done)
 *  4. PM2.5 sensor function-(Done)
 *  5. 3 LEDs
 *  6. pinMode(var, INPUT) <-- var is an INPUT (e.g. sensor/switch) vs digitalWrite(var, HIGH)-->var is set to HIGH or LOW (Done)
 *  7. pinMode all the other variables?? (Done)
 *  8. Important: SoftwareSerial, SIM800L.listen() or PM25.listen(); only one serial port at the time can listen; PM2.5 will listen first
 *  9. Sanity check for PM2.5>=1000 think what to do?--How to identify outliers in an array of 5 data points and substitute?
 *  10. In setup, GPS Enable pin? (Done)
 *  11  Average for Temp, RH (Done)
 *  12. Average for Battery (Done)
 *  13. State Machine for mobile mode--Important! (Done)
 *  14. SIM800L disconects if not enough current, (Burst is 2A) think if system should be activated after 95% Battery (Done), 
 *      SIM800L will not connect untill battery is above 90% level, will connect and disconect every try
 *  14a. Add a big capacitor to the SIM800L?
 *  15. SD card to save data-if simcad is not working, no sim card at all or any other issue, save data in a -SD card with RTC-
 *  16. Include an OLED screen
 *  17. Reduce number of variables O3avg, COavg... and O3array..
 *  18. within the STATE machine, include a Reset State and used inseted a power on/off state (Done)
 *  19. Use 30Hz (33msec) samplig rate for car, bicilce, walking and 1Hz (1sec) sampling rate for walking
 *  20. In mobile mode: sometimes, it doesn't diplay PM2.5 into the map, if the selection sensor reading starts with empty data e.g. [ND,10,20..20]-->will not plot this sequence in mobile mode
 *  21. however [10,20..20]--->this sequences will be ploted
 *  22. Mobile mode botton: disconect 43pin (14) and connect it to 3 pin (5-P6)
 *  New Updates Jan 2025
 *  23. Include Sanity Check functions to PM2.5, PM10, CO, O3, T y HR
 *  24. Implement RTOS for next SMAAs
************************************************************************************/
#include "DHT.h"
#include <LowPower.h>             // Low-Power library
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>       // Serial library
#include <stdio.h> 
#include <string.h> 
#include <math.h>



//Software UARTs
SoftwareSerial SIM800L(10,11);    //SIM800L Rx=10, Tx=11
SoftwareSerial HPMA115S0(12,13);  //PM2.5   Rx=12  Tx=13

//Hardware UARTs 
Adafruit_GPS GPS(&Serial2);       //GPS     Rx=17, Tx=16
HardwareSerial & ozone = Serial1; //Ozone   Rx=19, Tx=18
HardwareSerial & co = Serial3;    //CO      Rx=15, Tx=14

//Temp and RH sensor
#define DHTPIN A12
#define DHTTYPE DHT22              //DHT 22  (AM2302)
DHT dht(DHTPIN, DHTTYPE);

#define DataLed 37                 // Cellular Network 
#define GPSLed 33                  // GPS
#define BattLed 7                  // Battery

#define RG9_Pin 3                 //"Mobile" switch connected to digital pin 3; 0-->Fixed; 1-->Mobile
#define POWERSIM (4)               //Power SIM800L
#define ENGPS (5)                  //GPS Enable
#define POWER (9)                  //Enable power battery circuit
#define ANALOG_BATT_PIN (A14)      //Battery (%)
#define DATA_INTERVAL 20000

volatile byte DEVICE_STATE;         // volatile variable

double latitude, longitude;         // Define a default position?
double latitudeGPS,longituedGPS;

//float humidity;
//float temp;
        
const unsigned long GPS_TIMEOUT = 50000;  // Timeout in milliseconds
const unsigned long StateMachineinterval = 59000; //StateMachineInterval
const unsigned long StateMachineintervalMobile = 6500; //6500

unsigned long SIM_InitialTime;
unsigned long StateMachinetimer;
unsigned long StateMachinetimerMobile;
unsigned long gpsFixStartTime = 0;
 
//Sensor readings stored in two char arrays
char BattVal[4];                    // "80" = [0,1,2], size of the sting + 1
char TmpVal[7];                     // "0.99"= [0,1,2,3,4]-->[0,.,9,9,\0];
char RhVal[7];                      //
char OzoneVal[5];                   //
char COVal[5];                      //
char PM25Val[4];                    //
char PM10Val[4];                    //

char GPSVal[30];                    
// 25.709111,-100.303167
char GPSlat[15];
char GPSlong[15];

char Token[33] = ""; 
//char tx_SmabilityServer[500]; //Send buffer
char rx_SmabilityServer[150]; //Receive buffer
                         
char* serverName = "AT+HTTPPARA=\"URL\",\"http://smability.sidtecmx.com/Device//SetURL?deviceID=%s&batt=%s&temp=%s&hr=%s&pm25=%s&pm10=%s&o3=%s&co=%s&mode=0\"";
char* serverNameMobile = "AT+HTTPPARA=\"URL\",\"http://smability.sidtecmx.com/Device//SetURL?deviceID=%s&batt=%s&gps=%s&temp=%s&hr=%s&pm25=%s&pm10=%s&o3=%s&co=%s&mode=1\"";

bool flag = false;
bool flagGPS = false;               //initial state
bool flagFix = false;
bool HPMAstatus = false;
bool flagBatt = false;
bool flagGPSLED = false;
bool flagNet = false;

int value;
int mode;


volatile byte stateGPS=1;                     //active=1; innactive=0 
int O3;
int CO;
int pm25;
int pm10;
int counter=0;
int gpscounter=0;

float O3avg;
float COavg;
int PM25avg;
int PM10avg;

float Tmpavg;
float RHavg;

int O3array[3];
int COarray[3];
int PM25array[3];
int PM10array[3];

float Tmparray[3];
float RHarray[3];


void setup(){
  
  DEVICE_STATE = 0;                 //Inital Device STATE
  
  Serial.begin(57600);              //Terminal
  SIM800L.begin(9600);              //SIM800L
  
  delay(100);
  Serial.println("SMAA V1.0 Air Quality monitor | smability.io");
 
 
  HPMA115S0.begin(9600);             //PM2.5
  //while (!HPMA115S0);              // maybe is not a good idea to have an infinte loop
  start_autosendPM25();              //Configure HPMA115S0 sensor for automatic transmision; 
  
  if (start_autosendPM25()==1)
  {Serial.println("PM2.5 autosend");}
   
  ozone.begin(9600);                //O3; Rx=19, Tx=18
  
  //delay(100);
  if (commandQA() == 1)
  delay(100);
  { Serial.println("O3 in Q & A Mode"); }
  
  /*
  if (actUpload()==1)
  {Serial.println("O3 active upload");}
  */
  
  co.begin(9600);                   //co; Rx=15, Tx=14
  
  if (commandQACO()==1)
  delay(100);
  {Serial.println("CO in Q & A Mode");}
 
  /*
  if (actUploadCO()==1)
  delay(100);
  {Serial.println("CO active upload");}
  */
 
  GPS.begin(9600);                  //Initialize GPS at 9600 baud rate
  dht.begin();                      //temp and RH sensor
  //delay(100);
  
  pinMode(ANALOG_BATT_PIN, INPUT);   //Battery Analog PIN INPUT
  pinMode(POWERSIM,OUTPUT);          //SIM800L ON/OFF
  pinMode(ENGPS,OUTPUT);             //GPS enable
  pinMode(POWER,OUTPUT);             //Battery enable

  
  pinMode(DataLed, OUTPUT);          //Status GPS  
  pinMode(GPSLed, OUTPUT);           //Status GPS
  pinMode(BattLed, OUTPUT);          //Status Batt

  digitalWrite(ENGPS,LOW);           //Enable GPS with '0'
  delay(100);
  
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);  // Request RMC and GGA sentences
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);     // Set update rate to 1 Hz
  GPS.sendCommand(PGCMD_ANTENNA);
                                                
  powerOFFSIM();

  pinMode(RG9_Pin, INPUT);                    //Switch Mobile/Fixed to actived
  attachInterrupt(digitalPinToInterrupt(RG9_Pin), rgisr, LOW);

  blinkLEDs(4);                        // 3 LEDs blink two times
  delay(100);
  blinkLEDs(4);

}

// Interrrupt handler routine that is triggered when mobile switch is ON
// If the interrupt is active, rgisr() runs before the main void loop()

void rgisr() {
     DEVICE_STATE = 1;
}

void DEVICE_STATES(){
  
  switch (DEVICE_STATE)
  {
    
    case 0: //Fixed Mode
    
      //Serial.println(F(" DEVICE_STATE: 0 ")); // 1min sampling
      
      mode = 0;
      
      detachInterrupt (digitalPinToInterrupt (RG9_Pin)); 
      //SIM_STATES is the actual state machine 
      SIM_STATES(mode); //1 min sampling; An Interrupt rgisr() can occurt at this STATE at any time; hence--> DEVICE_STATE=1
      attachInterrupt(digitalPinToInterrupt(RG9_Pin), rgisr, LOW);
      /*--------------------------------------------------------------- 
      switch (stateGPS){ //use getGPSData(), returns 0 or 1
    
        case 0: // inactive GPS, send sensor data every min
          //Serial.println("GPS already found, 1 min wait");
          Serial.print("#"); //GPS already found, 1 min wait
          delay(1000); //think what to do, one line display
       
          break;
          
        case 1: // active GPS, if found, send location data only once
    
          GPSData(); // GPSData() should switch states; if returns = 1, GPSData=1 keep searching; else GPSData=0; GPS searching..

          break; 
      }
      */
      
      break;
   
    case 1: //Mobile Mode
    
      //Serial.println(F(" DEVICE_STATE: 1 "));
    
      mode = 1;
      
      detachInterrupt (digitalPinToInterrupt (RG9_Pin)); 
      SIM_STATES(mode);                                            //If the Interrupt is HIGH, How does it swiches back to Fixed mode state?
      attachInterrupt(digitalPinToInterrupt(RG9_Pin), rgisr, LOW); //Interrupt is Activated after a switch-case break
      
      DEVICE_STATE = 0; //Reset DEVICE_STATE to 0
   
      break;
  } 
 
}

void loop(){

  
  DEVICE_STATES();
  
  //this switch case is for fixed mode--only
  
  switch (stateGPS) //use getGPSData(), returns 0 or 1
  { 
    
    case 0: // inactive GPS, send sensor data every min, Serial.println("GPS already found, 1 min wait");
      
      detachInterrupt (digitalPinToInterrupt (RG9_Pin)); 
      noInterrupts();
      Serial.print("#"); //GPS already found, 1 min wait
      interrupts();
      delay(1000); //think what to do, one line display  
      // a--> use a non-blocking approach for instance LED ON and OFF for Mobile and Fixed mode <--
      // b--> validate sensor and display every sec
      
      attachInterrupt(digitalPinToInterrupt(RG9_Pin), rgisr, LOW);
      //attachInterrupt(digitalPinToInterrupt(RG9_Pin), rgisr, LOW); 
      break;
    case 1: // active GPS search
    
      detachInterrupt (digitalPinToInterrupt (RG9_Pin));     
      GPSData(); // GPSData() should switch states; if returns = 1, GPSData=1 keep searching; else GPSData=0; GPS searching..
      attachInterrupt(digitalPinToInterrupt(RG9_Pin), rgisr, LOW);

      break; 
  }

}


//SIM states
enum SIM
    {
      START,SENSORS,READSENSOR,
      PWRSIM,ATTACHGPRS,
      INITHTTP,SENDPARA,
      ENDGPRS,SIMOFF
    };

SIM SIM_state  = START;

void SIM_STATES(int state){
   
  int countloop = 0; // set countloop to 0
  
   
  switch (SIM_state)
  {
    
    case START:
     
      Serial.println(F("    STATE 0: START SENSORS "));
        
      readsensor();                                 //T&H 
           
      readPM2510();                                 //PM2.5/PM10 Concentration
      
      readO3QndA();                                 //O3 Concentration
                                 
      readCOQndA();                                 //CO Concentration

      battery();
     
      flag = false; 
            
      SIM_state = SENSORS;
        
      break; //goes to the closing brace (final brace) of switch-case body and continues with the next instruction-the interrupt-
      
    case SENSORS:
      
      Serial.println(F("    STATE 1: POWER ON SENSORS   "));
      
      if (state == 1){                                //Mobile mode

        SIM_state = READSENSOR;
        
      } else{                                         //Fixed mode (2)
          
        readsensor();                                 //Temp & RH sensor
        
        readPM2510();                                 //PM2.5 and PM10 Concentration
                                 
        readO3QndA();                                 //O3 Concentration
        
        readCOQndA();                                 //CO Concentration

        battery();
             
        SIM_state = PWRSIM;
      }
            
      break;

   case READSENSOR: 
    
      switch (state){ //use getGPSData(), returns 0 or 1
    
        case 0: //
        
          if ( (millis () - StateMachinetimer) >= StateMachineinterval){
            
             
             Serial.println(F("    STATE 2: READ SENSORS "));

             readsensor();                                //temp & RH sensor (3)

             readPM2510();                                //PM2.5 and PM10 Concentration

             //readO3();                                    
             
             readO3QndA();                                //Ozone Concentration
             
             //readCO();                                  

             readCOQndA();                                //CO Concentration
             
             battery();
             
             SIM_state = SENDPARA;

             StateMachinetimer = millis();
          }
       
          break;
        
        case 1: //
                    
          //2nd, 3th, 4th ..sensor measurement & 1st GPS data. (Read all sensor values in mobile mode)
         if ( (millis () - StateMachinetimerMobile) >= StateMachineintervalMobile){

             Serial.println(F("    STATE 2: READ SENSORS "));
          
            if (flag == true){ //mobile mode
        
              
              readsensor();                               //temp & RH sensor

              readPM2510();                               //PM2.5 and PM10 Concentration

              //readO3();                                 
              readO3QndA();                               //Ozone Concentration

              //readCO();                                   

              readCOQndA();                               //CO Concentration
              
              battery();
          
              SIM_state = SENDPARA;
           }
           else{ //          
                         
             /*
             readsensor();

             readPM2510();                                  //PM25 and PM10 Concentration
             
             //readO3();                                    //Ozone Concentration
             readO3QndA();
        
             readCO();                                       //CO concentration

             battery(); 
             */
             
             SIM_state = PWRSIM; //Turn On SIM800L
          }
          
          StateMachinetimerMobile = millis();
         }
          break;
      }

      break;
      
    case PWRSIM:
    
      
      Serial.println(F("    STATE 3: POWER ON SIM800L   "));

      SIM800L.listen();
    
      countloop = 0;                                            // rest while loop counter
      
      powerONSIM();                                             //Power ON SIM800L

      
      SIM_InitialTime = millis(); //Record SIM -ON- time
    
      //sendATcommand("AT+CSQ", "OK", 700);
      //delay(4000);
      
      //sendATcommand("AT+CGATT=1", "OK", 1000);
      //SIM is OFF-fly mode-minimun functionality
      
      delay(100);                 //wait some time
      
      sendATcommand("AT", "OK", 2000);
      //delay(500);
     
      //sendATcommand("AT+CSQ", "OK", 700);
     
      sendATcommand("AT+CFUN=1", "OK", 2000);

      //sendATcommand("AT+CIPSHUT", "OK", 1000); //de-Attach to GPRS network
      //delay(1000);
      
           
      sendATcommand("AT+CREG=1", "OK", 200);  // Activating CREG notifications
      //delay(1000);
      
      // All 0s means While is TRUE, countloop increments, we need at least one 1 to break the loop
      
      while ((sendATcommand("AT+CREG?", "+CREG: 1,1", 1000) // Registered, home network, GPRS
              || sendATcommand("AT+CREG?", "+CREG: 0,5", 1000) //registration, roaming
              || sendATcommand("AT+CREG?", "+CREG: 0,1", 1000)
              || sendATcommand("AT+CREG?", "+CREG: 2,1", 1000)
              || sendATcommand("AT+CREG?", "+CREG: 1,5", 1000)) != 1) //registerd, home network
              
              {countloop++; if(countloop >= 4){break;}};  // If it does not find network think what to do..

              
      sendATcommand("AT+CREG=0", "OK", 5000); // Deactivating CREG notifications; -->SIM gets stuck here!

                 
      //sendATcommand("AT+CSQ", "OK", 1000);
      //delay(2000);
                                  
      if (countloop < 4){ 
         
         SIM_state = ATTACHGPRS;
        
         Serial.println(F("Registered to the Network!"));
         
      }
      else{
          //SIM_state = SIMOFF;   //otherwise will hang-if not registered-, we will try again in 1 min
          SIM_state = ENDGPRS;    //insted of EDGPRS implement a Reset state?
          
          Serial.println(F("End GPRS, not Registered to the Network!"));
      }
            
      break;
      
    case ATTACHGPRS:
 
      Serial.println(F("    STATE 4: ATTACH GPRS    "));
   
      countloop = 0;             // reset while loop counter

      //start ATTACHGPRS state
      
      char aux_str[50]; 
      
      memset(aux_str, '\0', sizeof(aux_str)); //Why assign '\0' to all 50 memory locations?

      //sendATcommand("AT+CGATT=1", "OK", 2000); //Attach to GPRS network
      
      //delay(1000);
      //sendATcommand("AT+CGATT?", "OK", 1000); //Attach to GPRS network-->return ERROR
      //delay(1000);
      //sendATcommand("AT+CGATT=1", "OK", 25000);
      //sendATcommand("AT+SAPBR=0,1", "OK", 4000); // closing bearer if open
      
      sendATcommand("AT+SAPBR=3,1,\"Contype\",\"GPRS\"", "OK", 2000);
      
   
      //hologram
      
      snprintf(aux_str, sizeof(aux_str), "AT+SAPBR=3,1,\"APN\",\"%s\"", "hologram");
      sendATcommand(aux_str, "OK", 2000);
      
      
      //Telcel
      /*
      snprintf(aux_str, sizeof(aux_str), "AT+SAPBR=3,1,\"APN\",\"%s\"", "internet.itelcel.com");
      sendATcommand(aux_str, "OK", 2000);
      
      snprintf(aux_str, sizeof(aux_str), "AT+SAPBR=3,1,\"USER\",\"%s\"", "webgprs");
      sendATcommand(aux_str, "OK", 2000);
      
      snprintf(aux_str, sizeof(aux_str), "AT+SAPBR=3,1,\"PWD\",\"%s\"", "webgprs2002");
      sendATcommand(aux_str, "OK", 2000);
      */
        
           //+SAPBR: 1,1                                                       //before 6000-for both-gets stuck here!
      while ((sendATcommand("AT+SAPBR=1,1", "OK", 10000) || sendATcommand("AT+SAPBR=2,1", "+SAPBR: 1,1,\"", 10000)) != 1) // (1 != 1)--> (0)-->connected,  countloop = 0..or 1
              {countloop++;if(countloop >= 3){break;}}; // if it's (1), try again up to 10 times; countloop >= 5
             
      if  (countloop < 3){ //countloop < 5
        SIM_state = INITHTTP;
        
        Serial.println(F("Connected to the Network!")); //(0<5)-->(1)-->connected
        flagNet = true;     //Data LED steady ON
        blinkLEDs(2);       //Data LED blinking once
        
      }
      else{
        //Normal mode state
        SIM_state = ENDGPRS; //Can we go first to ENDGPRS then SIMOFF State?
        //SIM_state = SIMOFF;
        
        //SIM is OFF-fly mode-minimun functionality
       //sendATcommand("AT+CFUN=0", "OK", 1000);
       //SIM is ON
       //sendATcommand("AT+CFUN=1", "OK", 1000);
       
        Serial.println(F("End GPRS, Not Connected to the Network!"));
        
      }
      //end ATTACHGPRS state
      
      break;
      
    case INITHTTP:
     
      Serial.println(F("    STATE 5: INITHTTP   "));
      
      countloop = 0; // reset while loop counter
      
      //start INITHTTP
      
      while (sendATcommand("AT+HTTPINIT", "OK", 10000) != 1)
      {countloop++;if(countloop >= 3){break;}}; 
      
      delay(100);
      
      sendATcommand("AT+HTTPPARA=\"CID\",1", "OK", 5000);
      
      
      if  (countloop < 3){ //countloop < 3
        
        SIM_state = SENDPARA;
        
        Serial.println(F("Init HTTP functional!"));
        
      }
      else{
         //SIM_state =  INITHTTP; //try again-->how many times?, for both modes-->infinite loop(2)
         SIM_state = ENDGPRS; // or ENDGPRS session, for both modes?
         
         Serial.println(F("Init HTTP not functional!"));
         
      }
      //end INITHTTP
            
      break;
      
    case SENDPARA:
      
      Serial.println(F("    STATE 6: SENDPARA   "));
     
      SIM800L.listen();
      
      countloop = 0;                              // reset loop counter 
      
      char tx_SmabilityServer[400];               //auxiliary string -tx_SmabilityServer[300]-
     
      memset(tx_SmabilityServer, '\0', 400);      // with memory assignation of \0 to 300 locations?

      
      switch (state){                             //swtich-case; state=0 is fixed mode; state=1 is mobile mode
        
        case 0: //fixed mode:
          
          if (flagGPS == true){                   //if gps found, send device location 3 times
            
            if (gpscounter == 4){ 
              flagGPS=false;                      //disable flagGPS                                                            
              stateGPS=0;                         //disble GPS function 
              gpscounter=0;                       //reset gps counter 
            }
            
            gpscounter++;                         //gps counter
            
            //Token,BattVal,GPSVal,TmpVal,RhVal,PM25Val,PM10Val,OzoneVal,COVal
            snprintf(tx_SmabilityServer, sizeof(tx_SmabilityServer), serverNameMobile,Token,BattVal,GPSVal,TmpVal,RhVal,PM25Val,PM10Val,OzoneVal,COVal);
            
          }
          
          else{ // else send data every 1 min, no GPS data is send
     
          
            snprintf(tx_SmabilityServer, sizeof(tx_SmabilityServer), serverName,Token,BattVal,TmpVal,RhVal,PM25Val,PM10Val,OzoneVal,COVal);

          }
          
          break;
          
        case 1: //mobile mode

          stateGPS=1;          //report every 5sec-->Use 1 or 0 from gpsData()?

          //memset(tx_SmabilityServer, '\0', 500); //-->think if this line is necesarry
    
          snprintf(tx_SmabilityServer, sizeof(tx_SmabilityServer), serverNameMobile,Token,BattVal,GPSVal,TmpVal,RhVal,PM25Val,PM10Val,OzoneVal,COVal);
          
          
          break;
        }

      //AT command to be send
      //delay(3);
      sendATcommand(tx_SmabilityServer, "OK", 1000); // send URL, increase time to 600? org:520   2,601,0

      /* 
      dataSize = sizeof(tx_SmabilityServer)/sizeof(tx_SmabilityServer[0]);
      Serial.print("Array size: ");
      Serial.println(dataSize);
            
      dataSize *= sizeof(char*);
      Serial.print("Total byte size: ");
      Serial.println(dataSize);
      */


        
                                                                                             // AT+HTTPACTION=1 (POST), AT+HTTPACTION=0 (GET)
                                                                                             // AT+HTTPACTION=Method,StatusCode,DataLen 
                                                                                             // '3' can be removed--> 0,200
                                                                                         
      while ((sendATcommand("AT+HTTPACTION=0", "+HTTPACTION: 0,200,3", 15000)) != 1){           //Why 0,200,3 is the expected anwser? is 3 the amount of sent data in bytes?; AT+HTTPACTION=0"
       delay(3); // before = 3; this delay is important
       //delay(1000);
       countloop++;if(countloop >= 3){break;}; // we can try 5 times instead of 10           //+HTTPACTION: 0,200,1000 response where 200 is the OK and 1000 is the payload size.
      }

      if (countloop >= 3){ //countloop >= 10
          
          Serial.println(F("Failure in sending data"));
          
          SIM_state = ENDGPRS;
      }
      else{
       
        Serial.println(F("Success in sending data!"));
        // delay(1000);
        if(sendATcommand("AT+HTTPREAD", "ok!", 5000) == 1){   //before "!!"                                         // recevices: --ok!-- from Smability platform
          //+HTTPREAD: 3
          if (state == 1){
              blinkLEDs(2); //Data LED blinking every 15 sec
              blinkLEDs(2);
          } 
          else{ //state = 0
              blinkLEDs(2); //Data LED blinking every 1 min
              blinkLEDs(2);
              flagGPSLED = false; //GPS disconnected
          }
        }

        //if-else independent of ATcommand  
        if (state == 1){
          
          SIM_state = READSENSOR;
          flag = true;
         
          attachInterrupt(digitalPinToInterrupt(RG9_Pin), rgisr, LOW);
        }
        else{ //state = 0
          //SIM_state = ENDGPRS;
          SIM_state = READSENSOR;
          
        }
      }
       
      break;
      
    case ENDGPRS:
      
      Serial.println(F("    STATE 7: ENDGPRS    "));
     
      //start ENDGPRS
      
      sendATcommand("AT+HTTPTERM", "OK", 4000); // closing http if open
      sendATcommand("AT+CLTS=0", "OK", 2000); // "Get Local Time" Stamp Disabled
      //sendATcommand("AT+HTTPACTION=0", "OK", 2000); //?
      sendATcommand("AT+CFUN=1,1", "OK", 2000); //? 
      sendATcommand("AT+SAPBR=0,1", "OK", 4000); // closing bearer if open
      sendATcommand("AT+CGATT=0","OK",2000); //DE-Atach from the the network
      sendATcommand("AT+CIPSHUT", "OK", 2000); //De-Attach to GPRS network
      
      delay(1000);
      
      SIM_state = SIMOFF;
      
      break;
      
    case SIMOFF:
     
      Serial.println(F("    STATE 8: SIMOFF   "));
      
      
      //start SIMOFF
      
      countloop = 0; // while loop reset    
      
      while (sendATcommand("AT+CPOWD=1", "DOWN", 5000) != 1) 
        {countloop++; if(countloop >= 2){break;}};
     
      Serial.print(F("Elapsed Time in Seconds: "));
     
      Serial.println((millis() - SIM_InitialTime)/1000); // Display SIM ON time in seconds
     
      //end SIMOFF
      
      state = 0; //reset mode to fixed
      //powerOFFSensor();
      flagNet = false;    //Data disconnected
      blinkLEDs(2);
      flagGPSLED = false; //GPS disconnected
      blinkLEDs(1);
      
      powerOFFSIM(); //for now lets not switch off SIM800L
      
      Serial.flush();
      
      SIM_state = START;

      attachInterrupt(digitalPinToInterrupt(RG9_Pin), rgisr, LOW);
      
      break;
  }
  delay(1); //stability
}


/***********************************************************************************
 * Sleep Function
************************************************************************************/
void sleep(){
    attachInterrupt(digitalPinToInterrupt(RG9_Pin), rgisr, LOW);
    for (int i = 0; i < 35; i ++){ //7=1 min sleep approx; 420=1hr approx
      LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); 
    }
}

/***********************************************************************************
 * Blink LED Function
************************************************************************************/
void blinkLEDs(int LEDcomand) {
  const int ledPins[3] = {BattLed, GPSLed, DataLed}; // Array for LED pins
  const int delayTime = 250; // Milliseconds for delay (moved outside loop)

  switch (LEDcomand) {
    case 0:
    case 1:
    case 2:
      digitalWrite(ledPins[LEDcomand], HIGH);
      delay(delayTime);
      digitalWrite(ledPins[LEDcomand], LOW);
      delay(delayTime);
      break;
    case 4:
      // Turn all LEDs on
      for (int i = 0; i < sizeof(ledPins) / sizeof(ledPins[0]); i++) {
        digitalWrite(ledPins[i], HIGH);
      }
      delay(delayTime);

      // Turn all LEDs off (can be optimized further by using digitalWrite(port, value))
      for (int i = 0; i < sizeof(ledPins) / sizeof(ledPins[0]); i++) {
        digitalWrite(ledPins[i], LOW);
      }
      delay(delayTime);

      // Repeat on/off sequence twice
      for (int repeat = 0; repeat < 2; repeat++) {
        for (int i = 0; i < sizeof(ledPins) / sizeof(ledPins[0]); i++) {
          digitalWrite(ledPins[i], HIGH);
        }
        delay(delayTime);

        for (int i = 0; i < sizeof(ledPins) / sizeof(ledPins[0]); i++) {
          digitalWrite(ledPins[i], LOW);
        }
        delay(delayTime);
      }
      break;
  }

  // Set individual LEDs based on flags (outside switch)
  digitalWrite(BattLed, flagBatt);
  digitalWrite(GPSLed, flagGPSLED);
  digitalWrite(DataLed, flagNet);
}

/***********************************************************************************
 * Search GPS (2)
************************************************************************************/

bool getBestGPSData() { 
  
  for (int i = 0; i < 2; i++) {  // Loop 3 times
    while (!GPS.newNMEAreceived()) { //detect the completion of a new NMEA sentence
      char c = GPS.read(); // Reading Incoming Data; Continuously collect GPS data as it arrives 
    }
    GPS.parse(GPS.lastNMEA()); 
    
    /*
     * Once a new NMEA sentence is detected, the while loop exits, and the GPS.parse(GPS.lastNMEA()) 
     * function parses the sentence to extract relevant GPS information, 
     * such as latitude, longitude, and fix status.
     */
    if (GPS.fix) {
      // Store the latest valid latitude and longitude
      latitude = GPS.latitudeDegrees;
      longitude = GPS.longitudeDegrees;
      return true;
      break;
    } 
    else {
     // Handle timeout or other errors
     handleGPSTimeout();
     return false;
     //Serial.println("GPS fix failed in attempt " + String(i + 1));
    }
  }
}

/***********************************************************************************
 * Search GPS (1)
************************************************************************************/

bool getGPSData(){
  
  char c = GPS.read();        //Read incomming GPS data
  if(GPS.newNMEAreceived()){  // Check for new NMEA sentence
  
    if(!GPS.parse(GPS.lastNMEA())) { //Parse the sentence
      //return; // return TERMINATES the execution of the getGPSData function.
            // Any code following the "return" statement wothin this IF block
            // won't be executed.
            // In essence, this return; statement causes the getGPSData function to exit prematurely 
            //if the parsing of the NMEA sentence fails.

      return false;
    }
    else {
      if((GPS.fix) && (GPS.fixquality>=1) ){ //include: && (GPS.fixquality>=1)
        latitude = GPS.latitudeDegrees;
        longitude = GPS.longitudeDegrees;
        return true;
       } else {
        handleGPSTimeout();
        return false;
      }
   }
 }
 return false;
}

/***********************************************************************************
 * Search GPS (1.1)
************************************************************************************/
//other option is to implement a do-while loop with Timeout
/*
bool getGPSData(double &latitude, double &longitude){
  
  const int numReadings = 3;
  double latSum = 0;
  double lonSum = 0;
  int successfulReadings = 0;
  
  for (int i = 0; i < numReadings; i++) {
    if(GPS.newNMEAreceived()){              // Check for new NMEA sentence
      if(GPS.parse(GPS.lastNMEA())) {       //Parse the sentence
        if(GPS.fix){   //&& GPS.fixquality>=1
          latSum += GPS.latitudeDegrees;
          latSum += GPS.longitudeDegrees;
          successfulReadings++;
          } else {
            handleGPSTimeout();             
          }
        }
      }
   }
 
  if (successfulReadings > 0) {
    latitude = latSum / successfulReadings;
    longitude = lonSum / successfulReadings;
    return true;
  }
  
 return false;
}
*/
/***********************************************************************************
 * GPS Timeout handle
************************************************************************************/
void handleGPSTimeout() {
  
  if (gpsFixStartTime == 0) {
    
    gpsFixStartTime = millis();                           // Start timeout timer
    
  } else if (millis() - gpsFixStartTime >= GPS_TIMEOUT) { //30 sec
    
    Serial.println("GPS fix timeout");
    // Potential actions:
    digitalWrite(ENGPS, HIGH);                            //Disable GPS
    Serial2.end();                                        // Close serial communication
    Serial2.begin(9600);                                  // Reinitialize GPS module
    gpsFixStartTime = 0;                                  // Reset timeout timer
    digitalWrite(ENGPS, LOW);                             //Enable GPS
  }
}

/***********************************************************************************
 * Calculate Distance between Two GPS Points (Haversine formula)
 ************************************************************************************/
double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371e3; // Earth's radius in meters
  double dLat = radians(lat2 - lat1);
  double dLon = radians(lon2 - lon1);
  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(radians(lat1)) * cos(radians(lat2)) *
             sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return R * c;
}

/***********************************************************************************
 * GPS Data
************************************************************************************/
void GPSData(){
  
  static double prevLatitude = 0.0; // Store previous latitude for distance calculation
  static double prevLongitude = 0.0; // Store previous longitude for distance calculation
  
  static double zeroLatitude = 0.0;
  static double zeroLongitude = 0.0;
  
  static bool hasPreviousData = false; // Flag to indicate if previous data exists
  static bool ZeroLocation = true;
  
  //getGPSData(latitude, longitude)
  if(getGPSData()){ // if found report GPS data option 1: getGPSData(); option2:getBestGPSData()
    
    flagGPS = true; //set GPS flag
    flagGPSLED = true;

    memset(GPSlat,'\0', sizeof(GPSlat));
    memset(GPSlong,'\0', sizeof(GPSlong));
    memset(GPSVal,'\0', sizeof(GPSVal)); //clears the GPSVal array before using snprintf


    if (mode == 1) {      //mobile mode
      // Calculate distance if there's previous data
      double distance = 0.0;
      
      if (hasPreviousData) {
        distance = calculateDistance(prevLatitude, prevLongitude, latitude, longitude);
      }

      // Display data only if distance is significant (>= 2 meters)
      if (distance >= 1.5) {
        blinkLEDs(1); //blink GPS LED then steady
        Serial.println("Significant GPS change detected:");
        Serial.println("Latitude: " + String(latitude));
        Serial.println("Longitude: " + String(longitude));
        Serial.println("Distance: " + String(distance) + " meters");
        
        // Update previous data for next iteration
        prevLatitude = latitude;
        prevLongitude = longitude;
     
        dtostrf(prevLatitude, 10, 6, GPSlat);  //10 characters, 9 digits
        dtostrf(prevLongitude, 10, 6, GPSlong); //11 characters, 10 digits; buffer: [12]: "-100.303167\0"
        
        snprintf(GPSVal,sizeof(GPSVal),"%s,%s",GPSlat,GPSlong);

        
        ZeroLocation = true;                  //update new ZeroLocation with the latest lat, long
        
      } else {

        Serial.println("Distance: " + String(distance) + " meters");
        
        hasPreviousData = true;
        
        if (ZeroLocation){
          zeroLatitude = latitude;
          zeroLongitude = longitude;
          ZeroLocation = false;
        }
        
        dtostrf(zeroLatitude, 10, 6, GPSlat);  //10 characters, 9 digits
        dtostrf(zeroLongitude, 10, 6, GPSlong); //11 characters, 10 digits; buffer: [12]: "-100.303167\0"
        snprintf(GPSVal,sizeof(GPSVal),"%s,%s",GPSlat,GPSlong);
        
      }
      
    } else {                //fixed mode
      
      //GPS to string conversion.
      dtostrf(latitude, 10, 6, GPSlat);
      dtostrf(longitude, 10, 6, GPSlong);
      snprintf(GPSVal,sizeof(GPSVal),"%s,%s",GPSlat,GPSlong);
      
    }

  } else { 
    stateGPS = 1;          //if getGPSData() returns False
   }
}

/***********************************************************************************
 * SIM800L Network Registration
************************************************************************************/

int checkRegistration() {
  while ((sendATcommand("AT+CREG?", "+CREG: 1,1",1000)||sendATcommand("AT+CREG?", "+CREG: 0,1",1000)||sendATcommand("AT+CREG?", "+CREG: 0,5", 1000))!= 1){ 
    return 0;  // Not registered
    }
    return 1;  // Registered
}

/***********************************************************************************
 * Read Temperature and humidity values
************************************************************************************/

void readsensor(){
  bool TempMeasureReceived = false;
  for (int j = 0; j<2; j++){
  
    float humidity = dht.readHumidity();
    float temp = dht.readTemperature();

    // check if returns are valid, if they are NaN (not a number) then something went wrong!
    if (isnan(temp) || isnan(humidity)) {
      Serial.println(F("Failed to read from DHT"));
    }
    else{     
    //sanity check-avoid negative values
    if (temp < 0 || humidity < 0 ){
      temp = 0.00;
      humidity = 0.00;
    }
      if(j==1){
        Tmparray[0] = temp;
        RHarray[0] = humidity;
        TempMeasureReceived = true;
     }
  }
  delay(1); 
}
  //Tmpavg = avr(Tmparray, 1); // float aaverage of 2 samples
  //RHavg =  avr(RHarray, 1);
  if (TempMeasureReceived){ 
    
    noInterrupts();
    Serial.print("Temp: ");
    Serial.println(Tmparray[0],2); // in ºC
    Serial.print("Humidity: ");
    Serial.println(RHarray[0],2); //%
    interrupts();
    
    memset(TmpVal, '\0', sizeof(TmpVal));
    memset(RhVal, '\0', sizeof(RhVal));
    
    //convert temperature to string
    dtostrf(Tmparray[0], 4, 2, TmpVal);
    dtostrf(RHarray[0], 4, 2, RhVal);

    
  }
}

/***********************************************************************************
 * Read 1 sample of O3 
************************************************************************************/
void readO3(){
  bool thirdMeasureReceived = false;

  for (int j = 0; j<2; j++){
  
    if (!receiveGasConcentration()) {
  
      Serial.println("Cannot receive data from ZE12A-Ozone");
   
      return;  // return false and restart serial port for ozone or call active upload again?
    }
    else {
      
      if(j==1){
        //sanity check-avoid negative values
        if (O3 < 0  ){
          O3 = 0;
        }
        
        O3array[0] = O3;  //record the 3rd iteration
        thirdMeasureReceived = true;
      }
   }
  delay(1);
}
  // Average of 3 samples
  //O3avg = round(avr_GasConcen(O3array, 1));//float rounded to a whole number (int)
  
  if (thirdMeasureReceived) {
    
   noInterrupts();
   Serial.print("O3: ");
   Serial.println(O3array[0],0); // in ppb
   interrupts();
   
   memset(OzoneVal, '\0', sizeof(OzoneVal)); //memset; set a block of memory to a specific value efficiently
   itoa(O3array[0], OzoneVal, 10);           //itoa converts the integer 03avg into a character string
   
  }
   //itoa(O3avg, OzoneVal, 10); 
                              //string is placed in the buffer passed, which must be large enoguh to hold the ouput   
}

/***********************************************************************************
 * Read CO Gas Concentrarion
************************************************************************************/
void readCO(){
//setting CO sensor.. 
 bool secondMeasureReceived = false; 
  
  for (int j = 0; j<2; j++){
    
    if (!receiveCOGasConcentration()) {
   
      Serial.println("Cannot receive data from ZE12A-CO");
    
      return; // return false
    }
    else {
      
      if(j==1){
       //sanity check-avoid negative values
        if (CO < 0  ){
          CO = 0;
        }
        
        COarray[0] = CO; //record the 2rd iteration
        secondMeasureReceived = true;
      }  
   }
    delay(1);
 }
  // Average of 3 samples
   //COavg = round(avr_GasConcen(COarray, 1));//float rounded to a whole number (int)
   /*
   noInterrupts();
   Serial.print("CO: ");
   interrupts();
   
   noInterrupts();
   Serial.println(COavg, 0);  // Print 2 decimal places
   interrupts();
   */
  if (secondMeasureReceived) {
    
   Serial.print("CO: ");
   Serial.println(COarray[0],0); // in ppb
    
   memset(COVal, '\0', sizeof(COVal)); //memset; set a block of memory to a specific value efficiently
   //convert O3Average to string
   //dtostrf(CO, 5, 0, COVal); //check the format!!?;//check the format!!?
   itoa(COarray[0], COVal, 10); //itoa converts the integer 03avg into a character string
                              //string is placed in the buffer passed, which must be large enoguh to hold the ouput }
  }
}

/***********************************************************************************
 * Read PM2.5 & PM10 Concentrarion V1.01
************************************************************************************/
void readPM2510(){
  
  if (!receive_msrntPM25()) {

    Serial.println("Cannot receive data from HPMA115S0!");
  
    return; //to exit the loop in case data is not received
    
  } else {

   noInterrupts();
   Serial.print("PM25: ");
   Serial.println(pm25, 1);  // Print 0 decimal places
   Serial.print("PM10: ");
   Serial.println(pm10, 1);  // Print 0 decimal places
   interrupts();

   memset(PM25Val, '\0', sizeof(PM25Val)); //memset; set a block of memory to a specific value efficiently
   memset(PM10Val, '\0', sizeof(PM10Val)); //memset; set a block of memory to a specific value efficiently

    //snprintf (msg, 16, "%D", PM25);
   itoa(pm25, PM25Val, 10);
    //dtostrf(pm25, 5, 0, PM25Val);
       
    //snprintf (msg, 16, "%D", PM10);
   itoa(pm10, PM10Val, 10);
   }
}


/***********************************************************************************
 * Read PM2.5 & PM10 Concentrarion
************************************************************************************/
/*
void readPM2510(){
  
for (int j = 0; j<1; j++){
  
  if (!receive_msrntPM25()) {

    Serial.println("Cannot receive data from HPMA115S0!");
  
    return; //to exit the loop in case data is not received
   }
   else {
    PM25array[j] = pm25;  //store 5 samples 
    PM10array[j] = pm10;
   }
  delay(1);
}

   // Average of 5 samples
   PM25avg = round(avr_GasConcen(PM25array, 1));//float rounded to a whole number (int)
   // Average of 5 samples
   PM10avg = round(avr_GasConcen(PM10array, 1));//float rounded to a whole number (int)
   
   noInterrupts();
   Serial.print("PM25: ");
   Serial.println(PM25avg, 1);  // Print 0 decimal places
   Serial.print("PM10: ");
   Serial.println(PM10avg, 1);  // Print 0 decimal places
   interrupts();
   
   memset(PM25Val, '\0', sizeof(PM25Val)); //memset; set a block of memory to a specific value efficiently
   memset(PM10Val, '\0', sizeof(PM10Val)); //memset; set a block of memory to a specific value efficiently

    //snprintf (msg, 16, "%D", PM25);
   itoa(PM25avg, PM25Val, 10);
    //dtostrf(pm25, 5, 0, PM25Val);
       
    //snprintf (msg, 16, "%D", PM10);
   itoa(PM10avg, PM10Val, 10);
}
*/

/***********************************************************************************
 * Receive PM2.5/PM10 measurement
************************************************************************************/

bool receive_msrntPM25 (void)
{
  HPMA115S0.listen();

  //delay(100);
 
  while(HPMA115S0.available() < 32);
  
  byte HEAD0 = HPMA115S0.read();
  byte HEAD1 = HPMA115S0.read();
  
  while (HEAD0 != 0x42){
    
    if (HEAD1 == 0x42){
      HEAD0 = HEAD1;
      HEAD1 = HPMA115S0.read();
    } 
    else{
      HEAD0 = HPMA115S0.read();
      HEAD1 = HPMA115S0.read();
    }
    
  }
  if (HEAD0 == 0x42 && HEAD1 == 0x4D){
    byte LENH = HPMA115S0.read();
    byte LENL = HPMA115S0.read();
    byte Data0H = HPMA115S0.read();
    byte Data0L = HPMA115S0.read();
    byte Data1H = HPMA115S0.read();
    byte Data1L = HPMA115S0.read();
    byte Data2H = HPMA115S0.read();
    byte Data2L = HPMA115S0.read();
    byte Data3H = HPMA115S0.read();
    byte Data3L = HPMA115S0.read();
    byte Data4H = HPMA115S0.read();
    byte Data4L = HPMA115S0.read();
    byte Data5H = HPMA115S0.read();
    byte Data5L = HPMA115S0.read();
    byte Data6H = HPMA115S0.read();
    byte Data6L = HPMA115S0.read();
    byte Data7H = HPMA115S0.read();
    byte Data7L = HPMA115S0.read();
    byte Data8H = HPMA115S0.read();
    byte Data8L = HPMA115S0.read();
    byte Data9H = HPMA115S0.read();
    byte Data9L = HPMA115S0.read();
    byte Data10H = HPMA115S0.read();
    byte Data10L = HPMA115S0.read();
    byte Data11H = HPMA115S0.read();
    byte Data11L = HPMA115S0.read();
    byte Data12H = HPMA115S0.read();
    byte Data12L = HPMA115S0.read();
    byte CheckSumH = HPMA115S0.read();
    byte CheckSumL = HPMA115S0.read();
    if (((HEAD0 + HEAD1 + LENH + LENL + Data0H + Data0L + Data1H + Data1L + Data2H + Data2L + Data3H + Data3L + Data4H + Data4L + Data5H + Data5L + Data6H + Data6L + Data7H + Data7L + Data8H + Data8L + Data9H + Data9L + Data10H + Data10L + Data11H + Data11L + Data12H + Data12L) % 256) != CheckSumL){
      
      //Serial.println("PM25 sensor Checksum fail");
      
      return 0;
    }
    pm25 = (Data1H * 256) + Data1L;
    pm10 = (Data2H * 256) + Data2L;
    return 1;
  }
}

/***********************************************************************************
 * Enable Autosend PM2.5
************************************************************************************/

bool start_autosendPM25(void)
{
 // Start auto send
  HPMA115S0.listen();
 
  byte start_autosend[] = {0x68, 0x01, 0x40, 0x57};
  
  HPMA115S0.write(start_autosend, sizeof(start_autosend));
  
  HPMA115S0.flush();
  
  delay(500);
  
  //Then we wait for the response
  while(HPMA115S0.available() < 2);
  
  byte read1 = HPMA115S0.read();
  
  byte read2 = HPMA115S0.read();
  
  // Test the response
  if ((read1 == 0xA5) && (read2 == 0xA5)){
    // ACK
    return true;
  }
  else if ((read1 == 0x96) && (read2 == 0x96))
  {
    // NACK
    return false;
  }
  else return true;
}
  
/***********************************************************************************
 * Average Function for Gases and Particles
************************************************************************************/
float avr_GasConcen(int * GasConcen, int len){
  
  float ConcenAvg;
  long sum = 0L;
 
  for (int i = 0; i<len; i++){
    sum += GasConcen[i];
    delay(1);
  }
  
  return ConcenAvg = sum/len; //average will be fractional, round to int?
 }

/***********************************************************************************
 * Average Function for Temp & RH
************************************************************************************/
float avr(float * envMeasure, int len){
  
  float envAvg;
  float sum = 0.0;
 
  for (int i = 0; i<len; i++){
    sum += envMeasure[i];
  }
  
  return envAvg = sum/len; //average will be fractional, round to int?
 }
 
 
/***********************************************************************************
 * Battery Functions
************************************************************************************/
float BattVolt(){
  float voltage;
  int value;
  digitalWrite (POWER, HIGH);
  value = analogRead(ANALOG_BATT_PIN);
  voltage = (value * 5.00/1023)/0.5; //use this eq if R1= 100k and R2=100k
  //voltage = (value * 5.01/1023)/0.32; //use this eq if R1= 47k and R2=100k
  //digitalWrite (POWER, LOW);
  return voltage;
  }

float BattPercentage(){
  float Batpercentage;
  //Batpercentage = (BattVolt()-6)*30.3; // only for alkaline batteries; 9.3V Max
  //Batpercentage = ((BattVolt()-6)/(4.7))*100; //for AA alkaline and lithium batteries Max: 10.7v Min: 6.0v (pack of 6 AA batteries in series)

  Batpercentage = ((BattVolt()-3)/1.2)*100; //1.5v nominal value
  if (Batpercentage < 0) Batpercentage = 0; //do not display negative values
  return Batpercentage;
  }

void battery(){

  noInterrupts();
  Serial.print("Bat: ");
  Serial.println(BattPercentage()); // in %
  interrupts();
  
  //dtostrf(round(BattPercentage()),3,0,BattVal);
  memset(BattVal, '\0', sizeof(BattVal)); //memset; set a block of memory to a specific value efficiently
  itoa(round(BattPercentage()), BattVal, 10);

  
  if (round(BattPercentage()) <= 70){
    flagBatt = true;
    //blinkLEDs(0);//blink one time then remain steady
  }
  else{
    flagBatt = false;
    //blinkLEDs(0);
    digitalWrite(BattLed, flagBatt);
  } 
}
  
/***********************************************************************************
 * Power ON/OFF SIM800L
************************************************************************************/

void powerONSIM()
  {
  digitalWrite (POWERSIM, LOW);  // turn power ON
  delay (1000); // give time to power up    
  }  // end of powerOnPeripherals

void powerOFFSIM()
  {
  digitalWrite (POWERSIM, HIGH);  // turn power OFF
  delay (1000); // give time to power up    
  }  // end of powerOFFPeripherals

/***********************************************************************************
 * Default communication type: Active Upload-> Sends O3 gas concentration every second
************************************************************************************/
bool receiveGasConcentration(void){
    
    const size_t dataLength = 9;
    
    unsigned char sensor_response[dataLength];            // 9 Bytes; buffer to receive O3 readings

    
   if (ozone.available() > 0) {                           // check if Serial Port is available
    
     for (int i=0; i < dataLength; i++){
      
          sensor_response[i] = ozone.read();              // read sensor data and stored its data into a 9 byte array 
      } 
    
   delay(1);
    //O3 = 0x2A; SO2=0x2B
    if ( (CheckSum(sensor_response,dataLength) == sensor_response[8]) && (sensor_response[0] == 0xFF) && (sensor_response[1] == 0x2A) ){ // check sensor response sum and byte0 and byte1
           
        //Gas concentration value=concentration high byte*256+concentration low byte
      
        O3 = ((sensor_response[4]*256 + sensor_response[5])); // try gas concentration
        //Serial.println(O3);
        //delay(100);
        return true;
      } else {
        //Invalid data received 
        return false;
      }
   } else {
    // No data available on the serial port
    return false;
   }
}

/***********************************************************************************
 * Default communication type: Active Upload-> Sends CO gas concentration every second
************************************************************************************/
bool receiveCOGasConcentration(void){
    
    const size_t dataLength = 9;
    
    unsigned char sensor_response[dataLength];          // 9 Bytes; buffer to receive CO readings

    
   if (co.available() > 0){                             // check if Serial Port is available
    
    for (int i=0; i < dataLength; i++){
      
      sensor_response[i] = co.read();                   // read sensor data and stored its data into a 9 byte array 
      //delay(10);
      //Serial.println(sensor_response[i], HEX);         // print sensor values
    }
   delay(1);
   //co: 0x04; so2:0x2B
    if ( (CheckSum(sensor_response,dataLength) == sensor_response[8]) && (sensor_response[0] == 0xFF) && (sensor_response[1] == 0x04) ){ // check sensor response sum and byte0 and byte1
           
        //Gas concentration value=concentration high byte*256+concentration low byte
      
        CO = ((sensor_response[4]*256 + sensor_response[5])); //try full measurment
        //Serial.println(CO);
        //delay(100);
        return true;
        
      } else {
        //Invalid data received
        return false;
      }
   } else {
    // No data available on the serial port
    return false;
   }   
}

/***********************************************************************************
 * Communication type: Initiative Active upload  mode Ozone (2)
************************************************************************************/

bool actUpload(void)
{
  
  unsigned char actUP[]={0xFF,0x01,0x78,0x40,0x00,0x00,0x00,0x00,0x47};

  unsigned char serialBuffer[9];

  ozone.write(actUP, sizeof(actUP));                //Tx

  ozone.flush();
  delay(500);

  if (ozone.available() > 0){                       // check if Serial Port is available
    for (int i=0; i < 9; i++){
      serialBuffer[i] = ozone.read();              // Rx read (1) sensor data and stored its data into a 9 byte array; sensor response is the Answer 
    }
  }

  //CheckSum
  if ( (CheckSum(serialBuffer,sizeof(serialBuffer)) == 0x47) && (serialBuffer[0] == 0xFF) && (serialBuffer[1] == 0x01) ){
    //ACK
    return true;
  }
  else 
    return false; 
}

/***********************************************************************************
 * Communication type: Initiative Active upload  mode CO (2)
************************************************************************************/

bool actUploadCO(void)
{
  
  unsigned char actUPco[]={0xFF,0x01,0x78,0x40,0x00,0x00,0x00,0x00,0x47};

  unsigned char serialBuffer[9];

  co.write(actUPco, sizeof(actUPco));

  co.flush();
  delay(500);

  if (co.available() > 0){                         // check if Serial Port is available
    for (int i=0; i < 9; i++){
      serialBuffer[i] = co.read();              // read (1) sensor data and stored its data into a 9 byte array; sensor response is the Answer 
    }
  }
  
  //CheckSum
  if ( (CheckSum(serialBuffer,sizeof(serialBuffer)) == 0x47) && (serialBuffer[0] == 0xFF) && (serialBuffer[1] == 0x01) ){
    //ACK
    return true;
  }
  else 
    return false; 
}

/***********************************************************************************
 * Display O3 concentration in Q & A mode
************************************************************************************/

void readO3QndA(){
  if (!readO3QAmode()){ 
   Serial.println("Cannot receive data from ZE12A-Ozone");
   return;
  }
  else{
   //sanity check-avoid negative values
   if (O3 < 0  ){
      O3 = 0;
    }
      noInterrupts();
      Serial.print("O3: ");
      Serial.println(O3); // in ppb
      interrupts();
      
      memset(OzoneVal, '\0', sizeof(OzoneVal)); 
      itoa(O3, OzoneVal, 10); //itoa converts the integer 03avg into a character string
   }
}

/***********************************************************************************
 * Display CO concentration in Q & A mode
************************************************************************************/

void readCOQndA(){
  if (!readCOQAmode()){ 
   Serial.println("Cannot receive data from ZE12A-CO");
   return;
  }
  else{
   //sanity check-avoid negative values
   if (CO < 0  ){
      CO = 0;
    }
      noInterrupts();
      Serial.print("CO: ");
      Serial.println(CO); // in ppb
      interrupts();
      
      memset(COVal, '\0', sizeof(COVal)); 
      itoa(CO, COVal, 10); //itoa converts the integer 03avg into a character string
   }
}

/***********************************************************************************
 * (1) Send command to switch to Q&A mode for O3
************************************************************************************/
bool commandQA(void)
{

  const size_t dataLength = 9;
  
  byte serial_Buffer[dataLength];                                 // 9 Bytes; buffer to receive CO/O3 readings
  
  byte QAcommand[]={0xFF,0x01,0x78,0x41,0x00,0x00,0x00,0x00,0x46}; // switch to Q&A mode command

 
  ozone.write(QAcommand, sizeof(QAcommand));                     //Write-->Send (Tx); QnA mode command send

  ozone.flush();
  //serialFlushOzone();
  delay(500);

  if (ozone.available() > 0){                                    //Check if Serial Port is available
    for (int i=0; i < dataLength; i++){
      serial_Buffer[i] = ozone.read();                            //Read-->Receive (Rx); read sensor response and stored it into a 9 byte array  
    }
                                                             
  if ((CheckSum(serial_Buffer,sizeof(serial_Buffer)) == 0x46) && (serial_Buffer[0] == 0xFF) && (serial_Buffer[1] == 0x01) ){
    //ACK
    return true;
    } else {
      return false;
    }
  } else {
    return false;
  }
}

/***********************************************************************************
 * (2) Read O3 Gas Concentration
************************************************************************************/
bool readO3QAmode(void)
{
  
  const size_t dataLength = 9;

  byte sensor_response[dataLength]; 
  
  byte gasCon[]={0xFF,0x01,0x86,0x00,0x00,0x00,0x00,0x00,0x79}; //Read gas concentration command

  ozone.write(gasCon, sizeof(gasCon));                          //Write: Send (Tx); send read gas concentration command

  ozone.flush();
  delay(500);

  if (ozone.available() > 0){                                   // check if Serial Port is available
    for (int i=0; i < dataLength; i++){
      sensor_response[i] = ozone.read();                        //Read: Receive (Rx); read sensor response and stored it into a 9 byte array
    }

                                                                //Validate CheckSum (3) 
  if ((CheckSum(sensor_response,sizeof(sensor_response)) == sensor_response[8]) && (sensor_response[0] == 0xFF) && (sensor_response[1] == 0x86) ){
    //ACK
    O3 = ((sensor_response[6]*256 + sensor_response[7])); // try gas concentration
    return true;
    } else {
      return false;
    }
  } else {
    return false; 
  }
}

/***********************************************************************************
 * Clearing Serial Buffer
************************************************************************************/
void serialFlushOzone(){
  while(ozone.available() > 0) {
    char t = ozone.read();
  }
}

/***********************************************************************************
 * (1) Send command to switch to Q&A mode
************************************************************************************/
bool commandQACO(void)
{

  const size_t dataLength = 9;
  
  byte serial_Buffer[dataLength];                                 // 9 Bytes; buffer to receive CO/O3 readings
  
  byte QAcommandco[]={0xFF,0x01,0x78,0x41,0x00,0x00,0x00,0x00,0x46}; // switch to Q&A mode command

 
  co.write(QAcommandco, sizeof(QAcommandco));                     //Write-->Send (Tx); QnA mode command send

  co.flush();
  //serialFlushOzone();
  delay(500);

  if (co.available() > 0){                                    //Check if Serial Port is available
    for (int i=0; i < dataLength; i++){
      serial_Buffer[i] = co.read();                            //Read-->Receive (Rx); read sensor response and stored it into a 9 byte array 
    }
                                                             
  if ((CheckSum(serial_Buffer,sizeof(serial_Buffer)) == 0x46) && (serial_Buffer[0] == 0xFF) && (serial_Buffer[1] == 0x01) ){
    //ACK
    return true;
    } else {
      return false;
    }
  } else {
    return false;
  }
}

/***********************************************************************************
 * (2) Read CO Gas Concentration
************************************************************************************/
bool readCOQAmode(void)
{
  
  const size_t dataLength = 9;

  byte sensor_response[dataLength]; 
  
  byte gasCon[]={0xFF,0x01,0x86,0x00,0x00,0x00,0x00,0x00,0x79}; //Read gas concentration command

  co.write(gasCon, sizeof(gasCon));                          //Write: Send (Tx); send read gas concentration command

  co.flush();
  delay(500);

  if (co.available() > 0){                                   // check if Serial Port is available
    for (int i=0; i < dataLength; i++){
      sensor_response[i] = co.read();                        //Read: Receive (Rx); read sensor response and stored it into a 9 byte array
    }

                                                                //Validate CheckSum (3) 
  if ((CheckSum(sensor_response,sizeof(sensor_response)) == sensor_response[8]) && (sensor_response[0] == 0xFF) && (sensor_response[1] == 0x86) ){
    //ACK
    CO = ((sensor_response[6]*256 + sensor_response[7])); // try gas concentration
    return true;
    } else {
      return false;
    }
  } else {
    return false; 
  }
}

/***********************************************************************************
 * ZE12A CheckSum Function
************************************************************************************/
unsigned char CheckSum(unsigned char *i,unsigned char ln){

  unsigned char j, sumByte=0;
  
  i+=1;//increments pointer i by 1 (in HEX value), jumps to another memory location [0] and discards first byte: 0xFF; Byte =  8 bits
  
  for (j=0; j<ln-2; j++){
    
    sumByte = sumByte + *i; //it's making the sum through pointers, get value of pointer i (sumByte starts with byte 1--location 0 until byte 8--location 7)
           
    /* pointer [x], [1], [2], [3], [4], [5], [6], [7], [8]-->checkSum
     * byte[]={0xFF,0x04,0x04,0x00,0x00,0x00,0x30,0xD4,0xF4};
    */
    
    i++;//i=i+1 reasing i, jump to the next memory location
    //delay(10);
  }
  
  sumByte = (~sumByte)+1; //one's complement, "fliping bits", Why +1 ?--> to complete the CheckSum
  
  /* one's complemen
   * A = 0011 1100
   * ~A = 1100 0011
  */
  
  return(sumByte); //returns a byte
}

/***********************************************************************************
 * AT-COMMAD SIM800L Function
************************************************************************************/
int8_t sendATcommand(char* ATcommand,  char* expected_answer, unsigned int timeout){

    //SIM800L.listen();
    
    uint8_t x=0,  answer=0;
    char response[500];
    unsigned long previous;

    memset(response, '\0', 500);    // Initialize the string

    delay(10);
    
    while( SIM800L.available() > 0) SIM800L.read();               // Clean the input buffer
      
    //if (ATcommand[0] != '\0')
    //{
      noInterrupts();
      SIM800L.println(ATcommand);    // Send the AT command
      //Serial.println(ATcommand);
      interrupts();
    //}
    

    x = 0;
    previous = millis();


    // this loop waits for the answer
    do{
        if(SIM800L.available() != 0){
            // if there are data in the UART input buffer, reads it and checks for the asnwer
            response[x] = SIM800L.read();
            delay(1);
            x++;
            // check if the desired answer  is in the response of the module
            if (strstr(response, expected_answer) != NULL){
                answer = 1;
            } 
        }

    }
    // Waits for the asnwer with time out
    while((answer == 0) && ((millis() - previous) < timeout));
    
    
    noInterrupts();
    Serial.println(response);    // Send the AT command
    interrupts();
    
    /*
    if((millis() - previous) > timeout){
      noInterrupts();
      Serial.print(F("Time exceeded: "));
      Serial.println(millis() - previous);
      interrupts();
      }
    */
    return answer;
    
}

      
//Other network connection approach
      /*
      int retries = 4;
      while (retries > 0 && !checkRegistration()) {
      delay(1000);
      //Serial.println(retries);
      retries--;
      }

      if (retries == 0) {
      Serial.println("Registration failed!");
      SIM_state = ENDGPRS;
      }
      else {
      Serial.println("Registered to network!");
      SIM_state = ATTACHGPRS;
      }
      */
