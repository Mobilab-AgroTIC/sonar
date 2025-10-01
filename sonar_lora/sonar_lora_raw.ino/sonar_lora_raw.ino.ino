
#include "LoRaWanMinimal_APP.h"
#include "Arduino.h"
#include <algorithm>


//CLEFS A MODIFIER SELON TTN
const char* APP_EUI = "9879876987098097";                     
const char* DEV_EUI = "8765434576869768";                     
const char* APP_Key = "BDF3FEAE1DE6D4A664541EE09C4B87DE";

int temps = 200; // Indiquez dans cette ligne la fréquence d'envoi de données, en secondes. (Ne pas aller plus bas que 3minutes, soit 180sec)

uint16_t userChannelsMask[6]={ 0x00FF,0x0000,0x0000,0x0000,0x0000,0x0000 };
static uint8_t counter=0;
uint8_t lora_data[3];
uint8_t downlink ;

const int AppEUI_len = strlen(APP_EUI);
const int DevEUI_len = strlen(DEV_EUI);
const int AppKey_len = strlen(APP_Key);

byte AppEUI_clefConvertie[8];
byte DevEUI_clefConvertie[8];
byte AppKey_clefConvertie[16];

uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t devEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t appKey[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

const byte TRANS_PIN = GPIO2; // Broche TRANSISTOR
const byte TRIGGER_PIN = GPIO5; // Broche TRIGGER
const byte ECHO_PIN = GPIO4;    // Broche ECHO

const unsigned long MEASURE_TIMEOUT = 25000UL; // 25ms = ~8m 340m/s
const float SOUND_SPEED = 340.0 / 1000;

///////////////////////////////////////////////////
//Some utilities for going into low power mode
TimerEvent_t sleepTimer;
//Records whether our sleep/low power timer expired
bool sleepTimerExpired;

static void wakeUp()
{
  sleepTimerExpired=true;
}

static void lowPowerSleep(uint32_t sleeptime)
{
  sleepTimerExpired=false;
  TimerInit( &sleepTimer, &wakeUp );
  TimerSetValue( &sleepTimer, sleeptime );
  TimerStart( &sleepTimer );
  //Low power handler also gets interrupted by other timers
  //So wait until our timer had expired
  while (!sleepTimerExpired) lowPowerHandler();
  TimerStop( &sleepTimer );
}


void convertirClef(const char* clef, byte* clefConvertie, int longueur) {
    for (int i = 0; i < longueur; i += 2) {
        char byteStr[3] = {clef[i], clef[i + 1], '\0'};
        clefConvertie[i / 2] = strtol(byteStr, NULL, 16);
    }
}

void remplirTableau(uint8_t* tableau, byte* clefConvertie, int longueur) {
    for (int i = 0; i < longueur / 2; i++) {
        tableau[i] = clefConvertie[i];
    }
}
///////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);

  /* Initialise les broches */
  pinMode(TRANS_PIN, OUTPUT);
  pinMode(TRIGGER_PIN, OUTPUT);
  digitalWrite(TRIGGER_PIN, LOW); // La broche TRIGGER doit être à LOW au repos
  pinMode(ECHO_PIN, INPUT);

  convertirClef(APP_EUI, AppEUI_clefConvertie, AppEUI_len);
  convertirClef(DEV_EUI, DevEUI_clefConvertie, DevEUI_len);
  convertirClef(APP_Key, AppKey_clefConvertie, AppKey_len);
      
  remplirTableau(appEui, AppEUI_clefConvertie, AppEUI_len);
  remplirTableau(devEui, DevEUI_clefConvertie, DevEUI_len);
  remplirTableau(appKey, AppKey_clefConvertie, AppKey_len);


  
  if (ACTIVE_REGION==LORAMAC_REGION_AU915) {
    //TTN uses sub-band 2 in AU915
    LoRaWAN.setSubBand2();
  }
 
  LoRaWAN.begin(LORAWAN_CLASS, ACTIVE_REGION);
  
  //Enable ADR
  LoRaWAN.setAdaptiveDR(true);

  while (1) {
    Serial.print("Joining... ");
    LoRaWAN.joinOTAA(appEui, appKey, devEui);
    if (!LoRaWAN.isJoined()) {
      //In this example we just loop until we're joined, but you could
      //also go and start doing other things and try again later
      Serial.println("JOIN FAILED! Sleeping for 30 seconds");
      lowPowerSleep(30000);
    } else {
      Serial.println("JOINED");
      break;
    }
  }
}

///////////////////////////////////////////////////
void loop()
{
  delay(10);
  uint8_t voltage = getBatteryVoltage()/50; //Tension en %
  counter++;  
  digitalWrite(TRANS_PIN, HIGH);
  delay(1000);
  
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
  
  long measure = pulseIn(ECHO_PIN, HIGH, MEASURE_TIMEOUT);
  uint16_t distance = measure / 2.00 * SOUND_SPEED * 100;

  digitalWrite(TRANS_PIN, LOW);
  delay(1000);
  
  lora_data[0] = voltage;
  lora_data[1] = highByte(distance);
  lora_data[2] = lowByte(distance);
  //Now send the data. The parameters are "data size, data pointer, port, request ack"
  Serial.printf("\nSending packet with counter=%d\n", counter);
  //Here we send confirmed packed (ACK requested) only for the first five (remember there is a fair use policy)
  bool requestack=counter<5?true:false;
  if (LoRaWAN.send(sizeof(lora_data), lora_data, 1 , requestack)) {
    Serial.println("Send OK");
  } else {
    Serial.println("Send FAILED");
  }

 lowPowerSleep(temps*1000); 
}

///////////////////////////////////////////////////
//Example of handling downlink data
void downLinkDataHandle(McpsIndication_t *mcpsIndication)
{
  Serial.printf("Received downlink: %s, RXSIZE %d, PORT %d, DATA: ",mcpsIndication->RxSlot?"RXWIN2":"RXWIN1",mcpsIndication->BufferSize,mcpsIndication->Port);
  for(uint8_t i=0;i<mcpsIndication->BufferSize;i++) {
    Serial.printf("%02X",mcpsIndication->Buffer[i]);
  }
  Serial.println();
}
