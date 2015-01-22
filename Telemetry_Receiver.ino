


// RECEIVER


#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define CE_PIN    48
#define CSN_PIN  49

//  VCC 3.3V !!!      NOT 5V
//  CE to Arduino     pin 9
//  CSN to Arduino    pin 10
//  SCK to Arduino    pin 13
//  MOSI to Arduino   pin 11
//  MISO to Arduino   pin 12
//  IRQ               Unused





const uint64_t pipe = 0xE8E8F0F0E1LL;

RF24 radio(CE_PIN,CSN_PIN);

unsigned int x_in[1] = {777};
int latBig;
int latMed;
int longBig;
int longMed;

int height = 1;
int key[4] = {0};
double tiny[2] = {0,0};

void setup(){
  Serial.begin(9600);
  delay(1000);
  Serial.println("Nrf24L01 Receiever Starting");
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.openReadingPipe(1,pipe);
  radio.startListening();
}

void loop(){

  if (radio.available())
  {
    bool done = false;

    while (!done)
      done = radio.read(x_in, sizeof(x_in));
  }

  else
    Serial.println("No radio available");

//Serial.println(x_in[0]);


if (x_in[0] == 0)
    Serial.println(x_in[0]);
//    beta = 1000;

else if (x_in[0]/10000 == 1){
    latBig = x_in[0] % 10000;
    key[0] = 1;}
    
else if (x_in[0]/10000 == 2){
    latMed = x_in[0] % 20000;
    key[1] = 1;}
    
else if (x_in[0]/10000 == 3){
    longBig = x_in[0] % 30000;
    height = longBig - longBig/10*10 + 3;
    longBig /= 10;
    key[2] = 1;}
    
else if (x_in[0]/10000 == 4){
    longMed = x_in[0] % 40000;
    key[3] = 1;}
    
else if (x_in[0]/1000 == 50)
      Serial.println(x_in[0]);
  //    beta = 2000;
  //    Incoming = 0
  
else if (x_in[0]/1000 == 55)
      Serial.println(x_in[0]);
  //    beta = 2000;
  //    Incoming = 0
    
for (int i = 0; i<=3; i++)
{
    if (key[i] == 0)
      break;
    if (i == 3 & key[i] == 1)
    {
      tiny[0] = ((double)latBig*10000+latMed)/100000;
      tiny[1] = ((double)longBig*10000+longMed)/100000*-1;
      Serial.println("Full Coordinates");
      Serial.println(tiny[0],5);
      Serial.println(tiny[1],5);
      Serial.println(height);
      for (int ii = 0; ii<=3; ii++)
         key[ii] = 0;
    }
}
      

delay(20);


}








