

#include <math.h>

// TRANSMITTER

//-------GPS-------
#include <SoftwareSerial.h>
#include <TinyGPS.h>

TinyGPS gps;
//4800-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
SoftwareSerial ss(4, 3);

/*  Facing to the right 
 GND
 VIN
 RX
 TX
 GND
 IPPS Unused
 */

//-------Wireless-------
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define CE_PIN    9
#define CSN_PIN  10
RF24 radio(CE_PIN,CSN_PIN);

//  VCC 3.3V !!!      NOT 5V
//  CE to Arduino     pin 9
//  CSN to Arduino    pin 10
//  SCK to Arduino    pin 13
//  MOSI to Arduino   pin 11
//  MISO to Arduino   pin 12
//  IRQ               Unused



//------------------Start Code------------------

const uint64_t pipe = 0xE8E8F0F0E1LL;

double tiny[3];

int no_gps_signal;
int frameCounter = 0;
boolean Automation;
boolean Follow_Me;
unsigned int outBound[1] = {0};
int nextPackage = 1;
double packageTiny[2] = {0,0};
double num;

int latBig;
int latMed;
int longBig;
int longMed;
int height = 3;



void GETGPS(double x[]){

bool newData = false;

  while (ss.available())
  {
    char c = ss.read();
    if (gps.encode(c)) 
      newData = true;
  }

  if (newData)
  {
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon);

    x[0] = flat;
    x[1] = flon;
    x[2] = 1;
  }

  else 
    x[2] = 0;

}


void setup(){
  Serial.begin(115200);
  ss.begin(4800);

  pinMode(6,INPUT);
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
  
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(pipe);

}


void loop(){
   
  if (digitalRead(5) == 1) // Dip 2
    Follow_Me = true;
  else
    Follow_Me = false;
    
  if (digitalRead(6) == 1) // Dip 1
    Automation = true;
  else
    Automation = false;
    
  height = map(analogRead(A1),0,1023,0,9);
  
  
//  ---Aquire GPS Signal and Check--- 
  
  if (frameCounter % 5 == 0)
  {
    GETGPS(tiny);
    if (tiny[2] == 0)
      no_gps_signal++;
    else
      no_gps_signal = 0;
  }
  
    if (nextPackage == 1)
  {
    packageTiny[0] = fabs(tiny[0] * 100000);
    packageTiny[1] = fabs(tiny[1] * 100000);
    latBig = packageTiny[0]/10000;
    latMed = fmod(packageTiny[0],(double)latBig*10000);    
    longBig = packageTiny[1]/10000;
    longMed = fmod(packageTiny[1],(double)longBig*10000);
    longBig *= 10; 
    longBig += height;
  }
  
  
  if (nextPackage == 1)
    outBound[0] = 10000+latBig;
  
  else if (nextPackage == 2)
    outBound[0] = 20000+latMed;
  
  else if (nextPackage == 3)
    outBound[0] = 30000+longBig;
  
  else if (nextPackage == 4){
    outBound[0] = 40000+longMed;
    nextPackage = 0;}
  

  nextPackage++;

  
//  ---Determine Mode and Set up Package---
      
   if (Automation && Follow_Me)
        ;

   else if (!Automation && !Follow_Me)
     outBound[0] = 0;
     
   else if (Automation && !Follow_Me)
     outBound[0] = 50000+height;

    if (no_gps_signal > 50 && Follow_Me){
       outBound[0] = 55000;
       digitalWrite(8,HIGH);}
    
    else if(no_gps_signal < 50)
        digitalWrite(8,LOW);

    if (outBound[0] != 0)
        digitalWrite(7,HIGH);
    else 
        digitalWrite(7,LOW);
       

  radio.write(outBound, sizeof(outBound));
   
  frameCounter++;
  if (frameCounter == 100)
    frameCounter = 0;
    
    
//  Serial.println(outBound[0]);

//  Serial.println(tiny[0],5);
//  Serial.println(tiny[1],5);

//  Serial.println(latBig);
//  Serial.println(latMed);  
//  Serial.println(longBig);
//  Serial.println(longMed);
//  Serial.println(" "); 
 
}









