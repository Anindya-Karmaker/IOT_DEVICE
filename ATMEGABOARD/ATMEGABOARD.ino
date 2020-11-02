// include the library code:
#include <SPI.h>
#include <LiquidCrystal.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include "Adafruit_MAX31855.h"
#include <Adafruit_MAX31865.h>
#include <TinyGPS++.h>
#include <IRremote.h>
#include "EEPROMAnything.h"
#include <Adafruit_ADS1015.h>
Adafruit_ADS1115 ads;
int16_t adc0, adc1, adc2, adc3;
byte alloff=0;
byte startir=0;
int RECV_PIN = 12;
int repeat=0;
#define RREF      430.0
#define RNOMINAL  102.0
float temp1=0.0000;
float temp2=0.0000;
float temp3=0.0000;
int sensorprobe=0;
float longt=0.000005;
float latt=0.000005;
float ph=0;
byte present=0;
float temp1Thres=110.0000;//THRESHOLD VALUE FOR WARNING USER
float temp2Thres=110.0000;//THRESHOLD VALUE FOR WARNING USER
float sensorThres=4.8500;//THRESHOLD VALUE FOR WARNING USER
#define presence1 A2
#define presence2 A3
unsigned long instrumentsampling=millis();
float sensorvolt;
int bufph[10],tempph;
IRrecv irrecv(RECV_PIN);
IRsend irsend;
String esp32recv="                ";
char irtype='r';
decode_results results;
int codeType = -1; // The type of code
uint32_t codeValue=0; // The code value if not raw
uint16_t rawCodes[RAWBUF]={}; // The durations if raw
uint8_t codeLen=0; // The length of the code
int toggle = 0;
byte state=0;
typedef struct codetypeRAW{
  uint16_t rawCodes[RAWBUF];    // The data bits if type is not raw
  int codeLen;// DATA BITS  
};
codetypeRAW ircoderaw[4]={{}};
typedef struct codetype_KNOWN{
  uint8_t codeType;  // The type of code
  unsigned long codeValue;    // The data bits if type is not raw
  int codeLen; 
  int repeat;
};
codetype_KNOWN ircodek[6];
typedef struct thresholdValue{
  float temp1Thres;
  float temp2Thres;
  float sensorThres;
};
thresholdValue thresVal[1];
byte irbot=0;

TinyGPSPlus gps;
byte gpson=0;
byte wifion=0;
byte t_change=0;
const byte numChars = 90;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing
byte stw=0;
byte par=0;
byte s2=0;
byte m2=0;
byte h2=0;
byte dw2=0;
byte dd2=0;
byte mm2=0;
byte yy2=0; 
boolean newData = false;
#define rtc 0x68

#define TCprobe 3
#define RTprobe 4
//Adafruit_MAX31865 max = Adafruit_MAX31865(RTprobe, 5, 6, 7);
Adafruit_MAX31865 max = Adafruit_MAX31865(RTprobe);
//Adafruit_MAX31855 thermocouple(7, TCprobe, 6);
Adafruit_MAX31855 thermocouple(TCprobe);
SoftwareSerial gpsdev(2, 9);
SoftwareSerial miscdev(31, 9);//11 RX BLEDEV, 10 TX ESP32g 
typedef struct caldata{
  float cap_cal; 
  float tempS1_M;
  float tempS1_C;
  float tempS2_M;
  float tempS2_C;
  byte gps;
  byte wifi;
};
String date_time; //"1;12;12;45;78;10;11;12,";
char timeout[]="00:00:00";
char dateout[]="SUN 17 SEP 2017 ";
int calib=0;
#define buzzer A4
int high_low=0;
float supplyvolt=5.12;
float set_temp=0.0;
int cap_run=0;
int time_in=0;
byte maxopt=9;
byte play=1;
int i=50;


LiquidCrystal lcd(23,22,21,20,19,18);
byte charging[] = {
  B11111,
  B01111,
  B00110,
  B01100,
  B11110,
  B00100,
  B01000,
  B10000
};
byte tempChar[] = {
  B00100,
  B01010,
  B01010,
  B01110,
  B01110,
  B01110,
  B11111,
  B11111
};
byte celsius[] = {
  B10000,
  B00000,
  B01110,
  B10000,
  B10000,
  B10000,
  B01110,
  B00000
};
byte batteryfull[] = {
  B01110,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111
};
byte batteryhalf[] = {
  B01110,
  B10001,
  B10001,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111
};
byte batterylow[] = {
  B01110,
  B10001,
  B10001,
  B10001,
  B10001,
  B10001,
  B11111,
  B11111
};
byte gpsimg[] = {
  B00000,
  B10001,
  B11010,
  B11100,
  B11110,
  B11111,
  B10000,
  B10000
};

byte wifi[] = {
  B00000,
  B00000,
  B00000,
  B00111,
  B01000,
  B10011,
  B10100,
  B10101
};
int sn=5;
int value[5]={};
const int but[4]={1,0,14,30};
byte pressed=0;
int b=0;
int k;
unsigned long stat=0;
unsigned long a=0;
void readbutton(){
  b=0;
  if (digitalRead(but[0])==HIGH){k=1;}
  else if(digitalRead(but[1])==HIGH){k=2;}
  else if(digitalRead(but[2])==HIGH){k=3;}
  else if(digitalRead(but[3])==HIGH){k=4;}
  if (k!=0 && pressed==0){
    stat=millis();
    pressed=1;
  }
  if((millis()-stat)>=200 && pressed==1){
    b=k;
    k=0;
    pressed=0;
  }

}
float var=0;
byte set=0;
void buttonaction(){  
  switch (b){    
    case 1:
    set=0;
    var=0;
    present=0;
    startir=0;
    alloff=0;
    digitalWrite(buzzer,LOW);
    play=play+1;
    time_in=0;
    lcd.setCursor(0,0);
    lcd.print("                ");
    lcd.setCursor(0,1);
    lcd.print("                ");
    if(play>maxopt){
      play=1;        
    } 
    break;
    case 2:
    var=var-1;
    break;
    case 3:
    var=var+1;
    break;
    case 4:
    break;    
}
}
int  resolution = 12;
int  delayInMillis = 0;
byte key=0;
int storageSpace;
int storageSpace2;

void saveCodeKnown(codetype_KNOWN ircodek[],byte b){
  ircodek[b].codeType=codeType;
  ircodek[b].codeValue=codeValue;
  ircodek[b].codeLen=codeLen;
}
void saveCodeRaw(codetypeRAW ircoderaw[],byte b){
  ircoderaw[b].rawCodes[RAWBUF]=rawCodes[RAWBUF];//uint16_t rawCodes[RAWBUF];
  ircodek[b].codeLen=codeLen;
}
void saveThresholdValue(thresholdValue thresVal[],byte b){
  thresVal[b].temp1Thres=temp1Thres;
  thresVal[b].temp2Thres=temp2Thres;
  thresVal[b].sensorThres=sensorThres;
}

void saveCodeWithButton(){
  if(time_in==0){    
    lcd.setCursor(0,0);
    lcd.print(F("IR CODE SAVER"));
    lcd.setCursor(0,1);
    lcd.print(F("SLOT:"));
    lcd.setCursor(5,1);
    lcd.print(irbot);
    lcd.setCursor(6,1);
    lcd.print(F(" KEY:"));
    readbutton();
    buttonaction();
    if(b==4){
      irbot+=1;
      if(irbot>4){
        irbot=0;
      }
    }
    if(b==2 || b==3){
      irrecv.enableIRIn();
      key=b;
      b=0;
      time_in=1;
      lcd.setCursor(11,1);
      lcd.print(key);
      lcd.print(F("    "));
      delay(1000);
    }    
  }else if(time_in==1){  
    readbutton();
    buttonaction();     
    if (irrecv.decode(&results)&& state==0) {
      lcd.clear();
      state=1;
      irrecv.resume();// Receive the next value
    }
    if(state==1){
      storeCode(&results);      
      delay(1000);
      state=0;
      if(codeType==UNKNOWN){
      if(key!=1 && irbot==3){
      saveCodeRaw(ircoderaw, key-2);
      time_in=2;
      lcd.clear();
      }else if(key!=1 && irbot==4){
      saveCodeRaw(ircoderaw, key);
      time_in=2;
      lcd.clear();
      }else{
      lcd.setCursor(0,0);
      lcd.print(F("RAW CODE CAN BE"));
      lcd.setCursor(0,1);
      lcd.print(F("SAVED IN SLOT 2 "));
      delay(1000);
      time_in=0;
      lcd.clear();
     }
    }else if(codeType==NEC ||codeType==SONY || codeType==PANASONIC ||codeType==JVC ||codeType==RC5||codeType==RC6||codeType==SAMSUNG||codeType==LG){
     if(key!=1 && key!=4 && irbot!=3 or irbot!=4){
      if(irbot==0){
        saveCodeKnown(ircodek, key-2);
      }else if(irbot==1){
        saveCodeKnown(ircodek, key);
      }else if(irbot==2){
        saveCodeKnown(ircodek, key+2);
      }      
     time_in=2;
     lcd.clear();
     }else{
      lcd.setCursor(0,0);
      lcd.print(F("ONLY RAW CAN BE "));
      lcd.setCursor(0,1);
      lcd.print(F("SAVED IN SLOT 2"));
      delay(1000);
      time_in=0;
      lcd.clear();
     }     
    }
    }            
    }
    else if(time_in==2){
    
    lcd.setCursor(0,0);
    lcd.print(F("ENTER MORE ?   "));
    lcd.setCursor(0,1);
    lcd.print(F("THEN PRESS 4   "));
    readbutton();
    if(b==4){
      time_in=0;
      lcd.clear();
    }
    else if(b==2 || b==3){
      time_in=3;
      lcd.clear();
    }
  }else if(time_in==3){
    lcd.setCursor(0,0);
    lcd.print(F("SAVE TO EEPROM?"));
    lcd.setCursor(0,1);
    lcd.print(F("PRESS 2 to SAVE"));
    readbutton();
    buttonaction();    
    if(b==2){
      lcd.setCursor(0,0);
      lcd.print(F("SAVING..WAIT!  "));
      lcd.setCursor(0,1);
      lcd.print(F("                "));
      EEPROM_writeAnything(0,ircodek);
      storageSpace=sizeof(ircodek);
      EEPROM_writeAnything(storageSpace,ircoderaw);
      delay(500);
      lcd.setCursor(0,0);
      lcd.print(F("DONE SAVED     "));
      delay(1000);
      time_in=0;
      irbot=0;
    }    
  }
}

void storeCode(decode_results *results) {
  codeType = results->decode_type;
  int count = results->rawlen;
  if (codeType == UNKNOWN) {
    lcd.setCursor(0,0);
    lcd.print(F("RAW CODE"));
    codeLen = results->rawlen - 1; 
    for (int i = 1; i <= codeLen; i++) {
      if (i % 2) {
        // Mark
        rawCodes[i - 1] = results->rawbuf[i]*USECPERTICK - MARK_EXCESS;
        //lcd.print(" m");
      } 
      else {
        // Space
        rawCodes[i - 1] = results->rawbuf[i]*USECPERTICK + MARK_EXCESS;
        //lcd.print(" s");
      }
      //lcd.print(rawCodes[i - 1], DEC);
    }
    //lcd.print("");
   }
  else {
    if (codeType == NEC) {
      lcd.setCursor(0,0);
      lcd.print(F("NEC"));
      if (results->value == REPEAT) {
        
        // Don't record a NEC repeat value as that's useless.
        //lcd.print("repeat; ignoring.");
        return;
      }
    } 
    else if (codeType == SONY) {
      lcd.setCursor(0,0);
      lcd.print(F("SONY"));
    } 
    else if (codeType == PANASONIC) {
      lcd.setCursor(0,0);
      lcd.print(F("PANASONIC"));
    }
    else if (codeType == JVC) {
      lcd.setCursor(0,0);
      lcd.print(F("JVC"));
    }
    else if (codeType == RC5) {
      lcd.setCursor(0,0);
      lcd.print(F("RC5"));
    } 
    else if (codeType == RC6) {
      lcd.setCursor(0,0);
      lcd.print(F("RC6"));
    } 
    else if (codeType == SAMSUNG) {
      lcd.setCursor(0,0);
      lcd.print(F("SAMSUNG"));
    }
    else if (codeType == LG) {
      lcd.setCursor(0,0);
      lcd.print(F("LG"));
    }
    else {
      lcd.setCursor(0,0);
      lcd.print(F("UNEXPECTED ERROR"));
    }
    lcd.setCursor(0,1);
    lcd.print(results->value, HEX);
    lcd.setCursor(12,0);
    lcd.print(results->bits, DEC);
    codeValue = results->value;
    codeLen = results->bits;
    }
}


void sendCode(uint8_t codeType,unsigned int rawCodes[RAWBUF],unsigned long codeValue,int codeLen,int repeat) { 
  lcd.setCursor(0,1);
  lcd.print(F("               ")); 
  if (codeType == NEC) {
      irsend.sendNEC(codeValue,codeLen);
      lcd.setCursor(0,1);
      lcd.print(F("Sent NEC "));
      //lcd.print(codeValue, HEX);
    
  } 
  else if (codeType == SONY) {
    for (int i = 0; i < 3; i++) {
    irsend.sendSony(codeValue,codeLen);
    delay(40);
    }    
    lcd.setCursor(0,1);
    lcd.print(F("Sent Sony "));
    //lcd.print(ircodek[ti].codeValue, HEX);
  } 
  else if (codeType == PANASONIC) {
    irsend.sendPanasonic(codeValue,codeLen);
    lcd.setCursor(0,1);
    lcd.print(F("Sent Panasonic"));
    //lcd.print(ircodek[ti].codeValue, HEX);
  }
  else if (codeType == JVC) {
    irsend.sendJVC(codeValue, codeLen, false);
    lcd.setCursor(0,1);
    lcd.print(F("Sent JVC"));
    //lcd.print(ircodek[ti].codeValue, HEX);
  }
  else if (codeType == SAMSUNG) {
    irsend.sendSAMSUNG(codeValue,codeLen);
    lcd.setCursor(0,1);
    lcd.print(F("Sent SAMSUNG"));
    //lcd.print(ircodek[ti].codeValue, HEX);
  }
  else if (codeType == LG) {
    irsend.sendLG(codeValue,codeLen);
    lcd.setCursor(0,1);
    lcd.print(F("Sent LG"));
    //lcd.print(ircodek[ti].codeValue, HEX);
  }
  else if (codeType == RC5 || codeType == RC6) {
    int toggle;
    if (!repeat) {
      // Flip the toggle bit for a new button press
      toggle = 1 - toggle;
    }
    // Put the toggle bit into the code to send
    codeValue = codeValue & ~(1 << (codeLen - 1));
    codeValue = codeValue | (toggle << (codeLen - 1));
    if (codeType == RC5) {
      lcd.setCursor(0,1);
      lcd.print(F("Sent RC5 "));
      //lcd.print(codeValue, HEX);
      irsend.sendRC5(codeValue, codeLen);
    } 
    else {
      irsend.sendRC6(codeValue, codeLen);
      lcd.setCursor(0,1);
      lcd.print(F("Sent RC6 "));
      //lcd.print(ircodek[ti].codeValue, HEX);
    }
  } 
  else if (codeType == UNKNOWN /* i.e. raw */) {
    // Assume 38 KHz
    lcd.setCursor(0,1);
    irsend.sendRaw(rawCodes, codeLen, 38);
    lcd.print(F("Sent raw"));
  }
}
byte bcdtodec(byte val){
  return((val/16*10)+(val%16));
}
byte dectobcd(byte val){
  return((val/10*16)+(val%10));
}
void readTime(byte *second,byte *minute,byte *hour, byte *day,byte *date,byte *month,byte *year){
  Wire.beginTransmission(rtc);
  Wire.write(0);
  Wire.endTransmission();
  Wire.requestFrom(rtc,7);
  *second=bcdtodec(Wire.read()&0x7F);
  *minute=bcdtodec(Wire.read()&0x7F);
  *hour=bcdtodec(Wire.read()&0x3F);
  *day=bcdtodec(Wire.read())&0x7;
  *date=bcdtodec(Wire.read()&0x3F);
  *month=bcdtodec(Wire.read()&0x1F);
  *year=bcdtodec(Wire.read()&0xFF);  
}
void setTime(byte second,byte minute,byte hour, byte day,byte date,byte month,byte year){
  Wire.beginTransmission(rtc);
  Wire.write(0);
  Wire.write(dectobcd(second));
  Wire.write(dectobcd(minute));
  Wire.write(dectobcd(hour));
  Wire.write(dectobcd(day));
  Wire.write(dectobcd(date));
  Wire.write(dectobcd(month));
  Wire.write(dectobcd(year));
  Wire.endTransmission();
}
unsigned long lastbat=millis();
float outb=4.0;
byte lowbat=0;
void battery(){
  if(millis()-lastbat>1000){
    //analogRead(A1);
    adc0 = ads.readADC_SingleEnded(3);
    outb=(float)(adc0*0.1875/1000); 
    lastbat=millis();
  }  
  lcd.setCursor(15,0);  
  if (outb>=3.5 && outb<=4.3){
    lowbat=0;
    lcd.write(byte(2));
  }else if (outb>=3.3 && outb<3.5){
    lowbat=0;
    lcd.write(byte(3));
  }else if (outb>=2.8 && outb<3.3){
    wifion=0;
    digitalWrite(15,LOW);
    gpson=0;
    digitalWrite(A5,LOW);
    lowbat=0;
    lcd.write(byte(4));
  }else if (outb<2.8 && outb>=2){
    lcd.print(F("X"));
    wifion=0;
    digitalWrite(15,LOW);
    gpson=0;
    digitalWrite(A5,LOW);
    lowbat=1;
  }else if (outb>4.5){
    lowbat=0;
    wifion=1;
    digitalWrite(15,HIGH);
    gpson=1;
    digitalWrite(A5,HIGH);
    lcd.write(byte(5));
  }else if(outb<=2){
    gpson=1;
    digitalWrite(A5,HIGH);
  }
  if(lowbat==1){
    digitalWrite(buzzer, HIGH);
  }
   //EXTRA DEBUGGIN
}
unsigned long samplingtime=millis();
const char days[7][4]={{"SUN"},{"MON"},{"TUE"},{"WED"},{"THU"},{"FRI"},{"SAT"}};
const char months[12][4]={{"JAN"},{"FEB"},{"MAR"},{"APR"},{"MAY"},{"JUN"},{"JUL"},{"AUG"},{"SEP"},{"OCT"},{"NOV"},{"DEC"}};
byte second,minute,hour,day,date,month,year,ampm;
byte ss=0,mm=0,hh=0;

void displayTimeandTemp(){
  readTime(&second,&minute,&hour,&day,&date,&month,&year);  
  if (hour<12){    
    lcd.setCursor(8,0);
    lcd.print(F("AM   "));
  }else{
    lcd.setCursor(8,0);
    lcd.print(F("PM   "));
  }
  if (hour>12){
    hour=abs(hour-12);
  }else if(hour==0){
    hour=12;
  }
  lcd.setCursor(0,0);
  sprintf(timeout, "%02d:%02d:%02d", hour, minute, second);
  lcd.print(timeout);
  lcd.setCursor(0,1);
  sprintf(dateout, "%s %02d %s 20%02d ", days[day-1], date, months[month-1], year);//"SUN 17 SEP 2017"
  lcd.print(dateout);        
  if(millis()-samplingtime<=500){

  }else if(millis()-samplingtime>500 and millis()-samplingtime<1000){
      lcd.setCursor(2,0);
      lcd.print(F(" "));
      lcd.setCursor(5,0);
      lcd.print(F(" "));     
  }else if(millis()-samplingtime>=1000){
      samplingtime=millis();
  }
  if(wifion==1){
    lcd.setCursor(13,0);
    lcd.write(byte(6));
  }else if(wifion==0){
    lcd.setCursor(13,0);
    lcd.print(F(" "));
  }
  if(gpson==1){
    lcd.setCursor(14,0);
    lcd.write(byte(7));
  }else if(gpson==0){
    lcd.setCursor(14,0);
    lcd.print(F(" "));
  }
}

void setup() {
  EEPROM_readAnything(0,ircodek);
  storageSpace=sizeof(ircodek);
  EEPROM_readAnything(storageSpace,ircoderaw);
  storageSpace2=sizeof(ircoderaw);
  EEPROM_readAnything(storageSpace2,thresVal);
  temp1Thres=thresVal[0].temp1Thres;
  temp2Thres=thresVal[0].temp2Thres;
  sensorThres=thresVal[0].sensorThres;
  Serial.begin(9600);
  Serial1.begin(9600);
  gpsdev.begin(9600);
  miscdev.begin(9600);
  max.begin(MAX31865_2WIRE);
  pinMode(presence1, INPUT);
  pinMode(presence2, INPUT);
  pinMode(2, INPUT_PULLUP);
  pinMode(31, INPUT_PULLUP);
  pinMode(A5, OUTPUT);
  pinMode(15, OUTPUT);
  pinMode(buzzer,OUTPUT);
  for(int i=0; i<4;i++){
    pinMode(but[i], INPUT);
  }
  lcd.begin(16, 2);
  lcd.createChar(0, tempChar);
  lcd.createChar(1, celsius);
  lcd.createChar(2, batteryfull);
  lcd.createChar(3, batteryhalf);
  lcd.createChar(4, batterylow);
  lcd.createChar(5, charging);
  lcd.createChar(6, wifi);
  lcd.createChar(7, gpsimg);
  Wire.begin();
  ads.begin();
}

const int numReadings = 100;

int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
float average = 0;                // the average
int sampling(int pinvar){
 while(readIndex<numReadings){
  readings[readIndex]=analogRead(pinvar);
  delay(1);
  total=total+readings[readIndex];
  readIndex=readIndex+1;  
 }
 if(readIndex>=numReadings){
  readIndex=0;
 }
 average=float(total/numReadings);
 total=0;
 readIndex=0;
 return average;
}
byte s1_online=1;
byte s2_online=1;
void temp_read(){
  digitalWrite(RTprobe, LOW);
  temp1 = max.temperature(RNOMINAL, RREF);
  uint8_t fault = max.readFault();
  if (fault) {
    temp1=0.0000;
    s1_online=0;
    max.clearFault();
  }
  digitalWrite(RTprobe, HIGH);
  digitalWrite(TCprobe, LOW);
  temp2=thermocouple.readCelsius();
  if (isnan(temp2)) {
     temp2=0.0000;
     s2_online=0;
     
  }
  digitalWrite(TCprobe, HIGH);
  adc1 = ads.readADC_SingleEnded(1);
  temp3=(float)(adc1*0.1875/10);//sensorvolt
}
void gps_rx(){
  gpsdev.listen();
  while (gpsdev.available() > 0)
   if (gps.encode(gpsdev.read())){
    latt=(float)(gps.location.lat());
    longt=(float)(gps.location.lng());
   }
}
void send_ble(){
  //COMMON SERIAL SEND TO BOTH THE ESP32 and BLE MODULE
  //MODIFY PARAMATERS TO BE SENT. BE SURE TO ALSO MODIFY ESP32 RECEIVE PARAPMETERS!!
  Serial1.print(F("["));
  Serial1.print(outb,2);
  Serial1.print(F("|"));
  Serial1.print(temp1,2);
  Serial1.print(F("|"));
  Serial1.print(temp2,2);
  Serial1.print(F("|"));
  Serial1.print(sensorvolt,5);
  Serial1.print(F("|"));
  Serial1.print(present);
  Serial1.print(F("|"));
  Serial1.print(latt,6);
  Serial1.print(F("|"));
  Serial1.print(longt,6);
  Serial1.print(F("|"));
  Serial1.print(temp1Thres,2);
  Serial1.print(F("|"));
  Serial1.print(temp2Thres,2);
  Serial1.print(F("|"));
  Serial1.print(sensorThres,4);
  Serial1.print(F("]")); 
}

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;
    while (Serial1.available() > 0 && newData == false) {
        rc = Serial1.read();
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
    strtokIndx = strtok(tempChars, ";"); 
    par = (byte)(atoi(strtokIndx));     
    strtokIndx = strtok(NULL, ";");
    s2 = (byte)(atoi(strtokIndx));
    strtokIndx = strtok(NULL, ";");
    m2 = (byte)(atoi(strtokIndx));
    strtokIndx = strtok(NULL, ";");
    h2 = (byte)(atoi(strtokIndx));
    strtokIndx = strtok(NULL, ";");
    dw2 = (byte)(atoi(strtokIndx));
    strtokIndx = strtok(NULL, ";");
    dd2 = (byte)(atoi(strtokIndx));
    strtokIndx = strtok(NULL, ";");
    mm2 = (byte)(atoi(strtokIndx));
    strtokIndx = strtok(NULL, ";");
    yy2 = (byte)(atoi(strtokIndx));
    strtokIndx = strtok(NULL, ";");
    temp1Thres = (atof(strtokIndx));
    strtokIndx = strtok(NULL, ";");
    temp2Thres = (atof(strtokIndx));
    strtokIndx = strtok(NULL, ";");
    sensorThres = (atof(strtokIndx));
}
void receive_time(){
    recvWithStartEndMarkers();
    if (newData == true) {
        strcpy(tempChars, receivedChars);
        parseData();
        newData = false;
        time_in=2;
    }
}
//////////////
void parseThreshold() {      
    char * strtokIndx;
    strtokIndx = strtok(tempChars, ";"); 
    temp1Thres = (atof(strtokIndx));
    strtokIndx = strtok(NULL, ";");
    temp2Thres = (atof(strtokIndx));
    strtokIndx = strtok(NULL, ";");
    sensorThres = (atof(strtokIndx));
}
void receive_data(){
    recvWithStartEndMarkers();
    if (newData == true) {
        strcpy(tempChars, receivedChars);
        parseThreshold();
        newData = false;
        time_in=4;
    }
}
/////////////////////////
unsigned long presencesampling=millis();
void presence(){
  if(digitalRead(presence1)==HIGH || digitalRead(presence2)==HIGH){
    presencesampling=millis();
    present=1;
  }
  if(millis()-presencesampling>60000){
    present=0;
  }  
}
void serialFlush(){
  
  while(Serial1.available() > 0) {
    char tddua=Serial1.read();
  }
  Serial1.end();
  Serial1.begin(9600);
}  
byte alexaworks=0;
void alexa(){
  char command='?';
  if(Serial1.available()>0){
    command=(Serial1.read());
    switch(command){
      case 'a':
        sendCode(ircodek[0].codeType,0,ircodek[0].codeValue,ircodek[0].codeLen,ircodek[0].repeat);
        alexaworks=1;
        break;
      case 'b':
        sendCode(ircodek[1].codeType,0,ircodek[1].codeValue,ircodek[1].codeLen,ircodek[1].repeat);
        alexaworks=1;
        break;
      case 'c':
        sendCode(ircodek[2].codeType,0,ircodek[2].codeValue,ircodek[2].codeLen,ircodek[2].repeat);
        alexaworks=1;
        break;
      case 'd':
        sendCode(ircodek[3].codeType,0,ircodek[3].codeValue,ircodek[3].codeLen,ircodek[3].repeat);
        alexaworks=1;
        break;
      case 'e':
        sendCode(ircodek[4].codeType,0,ircodek[4].codeValue,ircodek[4].codeLen,ircodek[4].repeat);
        alexaworks=1;
        break;
      case 'f':
        sendCode(ircodek[5].codeType,0,ircodek[5].codeValue,ircodek[5].codeLen,ircodek[5].repeat);
        alexaworks=1;
        break;      
      case 'g':
        sendCode(UNKNOWN,ircoderaw[0].rawCodes,0,ircoderaw[0].codeLen,0);
        alexaworks=1;
        break;
      case 'h':
        sendCode(UNKNOWN,ircoderaw[1].rawCodes,0,ircoderaw[1].codeLen,0);
        alexaworks=1;
        break;
      case 'i':
        sendCode(UNKNOWN,ircoderaw[2].rawCodes,0,ircoderaw[2].codeLen,0);
        alexaworks=1;
        break;
      case 'j':
        sendCode(UNKNOWN,ircoderaw[3].rawCodes,0,ircoderaw[3].codeLen,0);
        alexaworks=1;
        break;
      case 'G':
        if(gpson==0){
          digitalWrite(A5,HIGH);
          gpson=1;
        }else{
          digitalWrite(A5,LOW);
          gpson=0;
        }
        break;
      case 'W':
        if(wifion==0){
          digitalWrite(15,HIGH);
          wifion=1;
        }else{
          digitalWrite(15,LOW);
          wifion=0;
        }
        break;        
      default: 
        break;     
    }
    serialFlush();
    if(alexaworks==1){
          lcd.setCursor(0,1);
          lcd.print(F("                "));
          lcd.setCursor(0,1);
          lcd.print(F("ALEXA COMMAND:"));
          lcd.print(command); 
          delay(500);
          lcd.print(F("                "));
          alexaworks=0;
    }
   
  }
}

void readph(){
     adc1 = ads.readADC_SingleEnded(1);
     sensorvolt=(float)(adc1*0.1875/1000);
     ph=sensorvolt*16.903-35.2575;//MODIFY ACCORDING TO THE TYPE OF PH SENSOR MODULE
}
void loop() {
  if(alloff==0){
    alexa();
    battery();
    presence();
    if(gpson==1){
      gps_rx();
    }
    if(millis()-instrumentsampling>1000){
      temp_read();
      readph();
      send_ble();
      instrumentsampling=millis();
    }
  }  
  switch(play){
  case 1:
  { 
    unsigned long ino,mili;
    int remsec;
    readbutton();
    buttonaction();
    if(time_in==0){
      displayTimeandTemp();
      if(b==4){
        time_in=1;
        lcd.setCursor(0,0);
        lcd.print(F("TIME CAL. MODE "));
        lcd.setCursor(0,1);
        lcd.print(F("                "));
        alloff=1;
      }else if(b==2){
        time_in=3;
        lcd.setCursor(0,0);
        lcd.print(F("DATA CAL. MODE "));
        lcd.setCursor(0,1);
        lcd.print(F("                "));
        alloff=1;
        }
    }else if(time_in==1){
      receive_time();
    }else if(time_in==2){
      lcd.setCursor(0,0);
      lcd.print(F("TIME RECEIVED "));
      lcd.setCursor(0,1);
      lcd.print(F("PRESS 2 TO SAVE"));
      if(b==2){
        alloff=1;        
        lcd.setCursor(0,1);
        lcd.print(F("SAVING WAIT    "));
        if(par==1){
            setTime(s2,m2,h2,dw2,dd2,mm2,yy2);
            delay(1000);
            lcd.setCursor(0,1);
            lcd.print(F("SAVED          ")); 
            delay(500); 
            lcd.setCursor(0,0);
            lcd.print(F("               "));
            lcd.setCursor(0,1);
            lcd.print(F("               "));        
        }
        par=0;
        t_change=0;  
        time_in=0;
        alloff=0;      
      }else if(b!=0 && b!=2){
        time_in=0;
        alloff=0;
      }
    }else if(time_in==3){
      receive_data();     
   }else if(time_in==4){
      alloff=1;
      lcd.setCursor(0,0);
      lcd.print(F("DATA RECEIVED "));
      lcd.setCursor(0,1);
      saveThresholdValue(thresVal,0);
      lcd.print(F("PRESS 2 TO SAVE"));
      if(b==2){        
        lcd.setCursor(0,1);
        lcd.print(F("SAVING WAIT    "));
        storageSpace2=sizeof(ircoderaw);
        EEPROM_writeAnything(storageSpace2,thresVal);
        delay(1000);
        lcd.setCursor(0,0);
        lcd.print(F("DONE SAVED     "));
        delay(1000);
        time_in=0; 
        alloff=0;     
      }else if(b!=0 && b!=2){
        time_in=0;
        alloff=0;
      }
   }
  break;
  }
  case 2:
  {
    readbutton();
    buttonaction();
    if(b==4){
      time_in+=1;
      if(time_in>1){
        time_in=0; 
      }
      lcd.setCursor(0,0);
      lcd.print(F("               "));
      lcd.setCursor(0,1);
      lcd.print(F("                "));
    }
    if(time_in==0){
      lcd.setCursor(0,0);
      lcd.print("S1.Temp S2.Temp");    
      lcd.setCursor(0, 1); 
      lcd.write(byte(0));
      if(s1_online=1){
        lcd.print(temp1,2);
      }else{
        lcd.print(F("S1 NA "));      
      }
      lcd.setCursor(7,1);
      lcd.write(byte(1));
      lcd.setCursor(8,1);
      lcd.write(byte(0));
      lcd.setCursor(9,1);
      if(s2_online=1){
        lcd.print(temp2,2);
      }else{
        lcd.print(F("S2 NA "));      
      }
      lcd.setCursor(15,1);
      lcd.write(byte(1));
    }else if(time_in==1){
      lcd.setCursor(0,0);
      lcd.print("Analog Temp.S3 "); 
      lcd.setCursor(0, 1); 
      lcd.write(byte(0));
      lcd.print(temp3,3);
      lcd.setCursor(7,1);
      lcd.write(byte(1));
      lcd.setCursor(8,1);
      lcd.print("        ");      
    }
    break;
  } 
    case 3:
  {
    float tempsense;
    readbutton();
    buttonaction();
    
    if(b==1){
      time_in=0;
      digitalWrite(buzzer,LOW);
    }
    if(b==2){
      set_temp=set_temp+0.1;
      digitalWrite(buzzer,LOW);
    }
    else if(b==3){
      set_temp=set_temp-0.1;
      digitalWrite(buzzer,LOW);
    }
    else if(b==4){
      if(set==0){        
        set=1;
      }else if(set==1){
        set=0;
        sensorprobe+=1;
        if(sensorprobe>2){
           sensorprobe=0;
        }
      }      
      time_in=0;
    }
    if(sensorprobe==0){
      lcd.setCursor(13,0);
      lcd.print(F("S1"));
      tempsense=temp1;
    }else if(sensorprobe==1){
      lcd.setCursor(13,0);
      lcd.print(F("S2"));
      tempsense=temp2;
    }else if(sensorprobe==2){
      lcd.setCursor(13,0);
      lcd.print(F("S3"));
      tempsense=temp3;
    }
    if (time_in==0){
       if(set==0){
        lcd.setCursor(0,0);
        lcd.print(F("HIGH ALARM   "));
        lcd.setCursor(0,1);
        lcd.print(F("               "));
        digitalWrite(buzzer, LOW);
        set_temp=tempsense+1.5;
       }else if(set==1){
        lcd.setCursor(0,0);
        lcd.print(F("LOW ALARM    "));
        lcd.setCursor(0,1);
        lcd.print(F("               "));
        digitalWrite(buzzer, LOW);
        set_temp=tempsense-1.5;
       }
       time_in=1;
    }
    lcd.setCursor(0,1);
    lcd.print(F("S:"));
    lcd.setCursor(2,1);
    lcd.print(set_temp,1);
    lcd.setCursor(8,1);
    lcd.print(F("C:"));
    lcd.setCursor(10,1);
    lcd.print(tempsense,1);  
    if(set==0){
      if(tempsense>=set_temp){
      digitalWrite(buzzer,HIGH);
      }
    }else if(set==1){
      if(tempsense<=set_temp){
      digitalWrite(buzzer,HIGH);
      }
    } 
    
    break;
  }
  case 4:
  {
    readbutton();
    buttonaction();
    presence();
    lcd.setCursor(0,0);
    lcd.print(F("DETECT HUMAN:  "));
    if(present==1){
      lcd.setCursor(0,1);
      lcd.print(F("DETECTED       "));
    }else{
      lcd.setCursor(0,1);
      lcd.print(F("NOT DETECTED   "));
    }
    break;
}
  case 5:
  { 
    readbutton();
    buttonaction();
    if(time_in==0){
      lcd.setCursor(0,0);
      lcd.print(F("pH METER       "));      
      lcd.setCursor(0,1);
      lcd.print(F("pH:"));
      lcd.print(ph);
      lcd.print(F("V:"));
      lcd.print(sensorvolt);
     }
     break; 
  }
  case 6:
  {
    readbutton();
    buttonaction();
    if(time_in==0){
      lcd.setCursor(0,0); 
      lcd.print(F("SETTINGS       "));
      lcd.setCursor(0,1);
      lcd.print(F("2.GPS 3.WIFI    "));
      if(b==2){
        time_in=1;
      }else if(b==3){
        time_in=2;
      }        
    }
    else if(time_in==1){
      if(gpson==0){          
      lcd.setCursor(0,0); 
      lcd.print(F("Turn GPS on?  "));
      lcd.setCursor(0,1);
      lcd.print(F("2.ON 3.OFF      "));
      if(b==2 && gpson==0){
        lcd.setCursor(0,0);
        lcd.print(F("                "));
        lcd.setCursor(0,1);
        lcd.print(F("                "));
        gpson=1;
        digitalWrite(A5,HIGH);
        lcd.setCursor(0,0);
        lcd.print(F("GPS ON"));
        delay(500);
        lcd.setCursor(0,0);
        lcd.print(F("               "));
        lcd.setCursor(0,1);
        lcd.print(F("                "));
      }
      }else if(gpson==1){
        gps_rx();
        lcd.setCursor(0,0);
        lcd.print(F("Lat:"));
        lcd.print(latt,6);
        lcd.setCursor(0,1);
        lcd.print(F("Lon:"));
        lcd.print(longt,6);
        if(b==3 && gpson==1){
          gpson=0;
          lcd.setCursor(0,0);
          lcd.print(F("                "));
          lcd.setCursor(0,1);
          lcd.print(F("                "));
          lcd.setCursor(0,0);
          lcd.print(F("GPS OFF        "));
          digitalWrite(A5,LOW);
          time_in=0;
        }
      }
    }else if(time_in==2){
        if(wifion==0){
        lcd.setCursor(0,0); 
        lcd.print(F("Turn WIFI on? "));
        lcd.setCursor(0,1);
        lcd.print(F("2.ON 3.OFF      "));
        if(b==2 && wifion==0){
          lcd.setCursor(0,0);
          lcd.print(F("                "));
          lcd.setCursor(0,1);
          lcd.print(F("                "));
          wifion=1;
          digitalWrite(15,HIGH);
          lcd.setCursor(0,0);
          lcd.print(F("WIFI ON"));
          delay(500);
          lcd.setCursor(0,0);
          lcd.print(F("               "));
          lcd.setCursor(0,1);
          lcd.print(F("                "));
        }
        }else if(wifion==1){
          lcd.setCursor(0,0);
          lcd.print(F("WIFI MODE      "));
          while (Serial1.available() > 0 && b!=3 && b!=1) {
            lcd.setCursor(0,1);
            esp32recv=Serial1.read();
            lcd.print(esp32recv);
          }
          if(b==3 && wifion==1){
            wifion=0;
            lcd.setCursor(0,0);
            lcd.print(F("                "));
            lcd.setCursor(0,1);
            lcd.print(F("                "));
            lcd.setCursor(0,0);
            lcd.print(F("WIFI OFF       "));
            digitalWrite(15,LOW);
            delay(500);
            time_in=0;
          }
    }
    }
    break; 
  }
  case 7:
  {
    readbutton();
    buttonaction();
    alloff=1;
    saveCodeWithButton();
    break;
  }
  case 8:
  {
    readbutton();
    buttonaction();
    lcd.setCursor(0,0);
    if(startir==0){
      lcd.setCursor(0,0);
      lcd.print(F("                "));
      lcd.setCursor(0,0);
      lcd.print(F("IR SENDER SLT:"));
      lcd.setCursor(14,0);
      lcd.print(irbot);
      lcd.setCursor(0,1);
      lcd.print(F("                "));
      startir=1;
    }
    if(b==4){
      irbot+=1;
      if(irbot>4){
        irbot=0;
      }
      lcd.setCursor(14,0);
      lcd.print(irbot);
    }
    if(b==2 or b==3){
      if(irbot==0){
        sendCode(ircodek[b-2].codeType,0,ircodek[b-2].codeValue,ircodek[b-2].codeLen,ircodek[b-2].repeat);
      }else if(irbot==1){
        sendCode(ircodek[b].codeType,0,ircodek[b].codeValue,ircodek[b].codeLen,ircodek[b].repeat);
      }else if(irbot==2){
        sendCode(ircodek[b+2].codeType,0,ircodek[b+2].codeValue,ircodek[b+2].codeLen,ircodek[b+2].repeat);
      }else if(irbot==3){
        sendCode(UNKNOWN,ircoderaw[b-2].rawCodes,0,ircoderaw[b-2].codeLen,0);
      }else if(irbot==4){
        sendCode(UNKNOWN,ircoderaw[b].rawCodes,0,ircoderaw[b].codeLen,0);
      }
    }
    break;
  }
  case 9:
  {
    readbutton();
    buttonaction();
    int value1;
    float out1;
    int analogVolt;
    lcd.setCursor(0,0);
    lcd.print(F("BAT VOLT:"));   
    lcd.print(outb);
    lcd.setCursor(0,1);    
    lcd.print(F("Analog volt "));
    lcd.print(sensorvolt);
    
  } 
 }
}
