#include <IOXhop_FirebaseESP32.h>
#include <IOXhop_FirebaseStream.h>
#include "ESP32_MailClient.h"
#include <Arduino.h>
#include <ESP32Ping.h>
#include <WiFi.h>
#include <DNSServer.h>
#include <WebServer.h>
#include <WiFiManager.h>   
     
// ------------------Modify according to your needs-----------------------------
#define FIREBASE_HOST "xyzabcd.firebaseio.com"   				///ENTER THE FIREBASE HOST NAME
#define FIREBASE_AUTH "abcdefghIjklMNOP"						///ENTER THE FIREBASE AUTHORIZATION CODE
#define SERIAL_BAUDRATE     9600
#define emailSenderAccount    "your_email_id_here@gmail.com"    ///ALSO ENABLE LESS SECURE APPS
#define emailSenderPassword   "password_here"
#define emailRecipient        "sender_email@gmail.com"			///ENTER RECIPIENT 1 EMAIL ID
#define emailRecipient2       "sender_email2@gmail.com"			///ENTER RECIPIENT 2 EMAIL ID
#define smtpServer            "smtp.gmail.com" 					///GOOGLE SMPT DATA FOR OTHER EMAIL ID
#define smtpServerPort        465								///GOOGLE SMPT DATA FOR OTHER EMAIL ID
#define emailSubject          "IoT Device Alert"  
const char* ntpServer = "pool.ntp.org"; //NOT CURRENTLY IMPLEMENTED
const long gmtOffset_sec = 0;           //NOT CURRENTLY IMPLEMENTED
const int daylightOffset_sec = 0;       //NOT CURRENTLY IMPLEMENTED
// -----------------------------------------------------------------------------
SMTPData smtpData;
int emailtime=0;
int loctime=0;
int to_hours;
int to_seconds_remaining;
int to_minutes;
int to_seconds;
unsigned int timestamp=0;
char data;
byte initial1=0;
byte warnuser=0;
byte wait=0;
int messagecount=0;

// -----------------------------------------------------------------------------
// Wifi
// -----------------------------------------------------------------------------
const byte numChars = 75;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing
float outb=0;
float t1=0;
float t2=0;
float sensorval=0;
byte pr=0;
float temp1Thres=110.0000;//THRESHOLD VALUE FOR WARNING USER
float temp2Thres=110.0000;//THRESHOLD VALUE FOR WARNING USER
float sensorThres=2.0000;//THRESHOLD VALUE FOR WARNING USER
double latt=0.000005;
double longt=0.000005;
const char* remote_host = "www.google.com";
boolean newData = false;
void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '[';
    char endMarker = ']';
    char rc;
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;

            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}
void parseData() {     
    char * strtokIndx; 
    strtokIndx = strtok(tempChars, "|"); 
    outb = (atof(strtokIndx));
    strtokIndx = strtok(NULL, "|");
    t1 = (atof(strtokIndx));    
    strtokIndx = strtok(NULL, "|");
    t2 = (atof(strtokIndx));
    strtokIndx = strtok(NULL, "|");
    sensorval = (atof(strtokIndx));
    strtokIndx = strtok(NULL, "|");
    pr = (byte)(atoi(strtokIndx));
    strtokIndx = strtok(NULL, "|");
    latt = (atof(strtokIndx));
    strtokIndx = strtok(NULL, "|");
    longt = (atof(strtokIndx));
    strtokIndx = strtok(NULL, "|");
    temp1Thres = (atof(strtokIndx));
    strtokIndx = strtok(NULL, "|");
    temp2Thres = (atof(strtokIndx));
    strtokIndx = strtok(NULL, "|");
    sensorThres = (atof(strtokIndx));
}
void receive_data(){
    recvWithStartEndMarkers();
    if (newData == true) {
        strcpy(tempChars, receivedChars);
        parseData();
        newData = false;
    }
}

void configModeCallback (WiFiManager *myWiFiManager)
{
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  Serial.println(myWiFiManager->getConfigPortalSSID());
}
void setup() {
  Serial.end();
  
  Serial.println("WAITING HERE");
  WiFiManager wifiManager;
  wifiManager.autoConnect("IOT_DEVICE","jv159357");
  Serial.println("connected...yeey :)");
}
unsigned long waittime=millis();
unsigned long last=millis();
bool checknet=false;
float temp2=22.22;
byte trynet=0;
void sendCallback(SendStatus info);
void loop() {

    if(initial1==0 && millis()-last>5000){
      last=millis();
      checknet=Ping.ping(remote_host);
      if(checknet) {
      Serial.println("Success!!");
      initial1=1;
      trynet=0;
      } else {
        initial1=0;
        Serial.println("Error");
        trynet+=1;
      }
      if(trynet>100){
        ESP.restart();
      }
    }else if(initial1==1) {
        Serial.println(F("WIFI CONNECTED"));
        Serial.begin(9600);         
        configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
        initial1=2;               
    }else if (initial1==2){
        timefunc(); 
        loctime=to_minutes;
        receive_data();
        if(millis()-timestamp>2500){
          Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
          String variablesstring="";
          variablesstring=variablesstring+"<"+String(outb,2)+"|"+String(t1,2)+"|"+String(t2,2)+"|"+String(sensorval,4)+"|"+pr+"|"+String(latt,6)+"|"+String(longt,6)+"|"+String(temp1Thres,2)+"|"+String(temp2Thres,2)+"|"+String(sensorThres,4)+"|"+String(to_seconds)+"|"+String(loctime)+"|"+String(emailtime)+">";
          Firebase.setString("variableout",variablesstring);
          timestamp=millis();
          warnuserfunc();
          if(warnuser==1 and wait==0){
            messagecount+=1;  
            emailtime=to_minutes;  
            smtpData.setLogin(smtpServer, smtpServerPort, emailSenderAccount, emailSenderPassword);
            smtpData.setSender("ESP32", emailSenderAccount);
            smtpData.setPriority("High");
            String subject=String(emailSubject)+" "+ String(messagecount);
            smtpData.setSubject(subject);
            String emailtopic="WARNING! THRESHOLD EXCEEDED! People Detected:"+String(pr)+" Temp1:"+String(t1,2)+" Temp2:"+String(t2,2)+" sensorval:"+String(sensorval,4)+" Lattitude:"+String(latt,6)+" Longtitude:"+String(longt,6);
            smtpData.setMessage("<div style=\"color:#2f4468;\"><h1>"+emailtopic+"</h1><p>- Sent from IoT board</p></div>", true);
            smtpData.addRecipient(emailRecipient);
			//smtpData.addRecipient(emailRecipient2);//FOR ADDING SECOND RECIPIENT
            MailClient.sendMail(smtpData);
            wait=1;
            warnuser=0;
            smtpData.empty();
          }          
        }
        if(abs(loctime-emailtime)>=5 and wait==1){   //5 min. is the time to wait before sending another email
            wait=0;
        }        
    }
}
void sendCallback(SendStatus msg) {
}
void warnuserfunc(){
  if(t1>temp1Thres or t2>temp2Thres or sensorval>sensorThres or pr==1){
    warnuser=1;
  }else{
    warnuser=0;
  }
}
void timefunc(){
  unsigned long to_Millis= millis();
  unsigned long to_to_seconds=to_Millis/1000;
  to_hours= to_to_seconds/3600;
  to_seconds_remaining=to_to_seconds%3600;
  to_minutes=to_seconds_remaining/60;
  to_seconds=to_seconds_remaining%60;
  return;
}
