/**
 * This is an example of joining, sending and receiving data via LoRaWAN using a more minimal interface.
 * 
 * The example is configured for OTAA, set your keys into the variables below.
 * 
 * The example will upload a counter value periodically, and will print any downlink messages.
 * 
 * please disable AT_SUPPORT in tools menu
 *
 * David Brodrick.
 */
#include "LoRaWanMinimal_APP.h"
#include "Arduino.h"
#include <algorithm>


/*
 * set LoraWan_RGB to Active,the RGB active in loraWan
 * RGB red means sending;
 * RGB purple means joined done;
 * RGB blue means RxWindow1;
 * RGB yellow means RxWindow2;
 * RGB green means received done;
 */

//Set these OTAA parameters to match your app/node in TTN
static uint8_t devEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0x57, 0x0F };
static uint8_t appEui[] = { 0xA8, 0x40, 0x41, 0x00, 0x00, 0x00, 0x01, 0x01 };
static uint8_t appKey[] = { 0x14, 0xC7, 0x60, 0xDD, 0xE7, 0xC3, 0x91, 0xAA, 0x0A, 0x6F, 0x8C, 0xAD, 0x92, 0x52, 0x29, 0x82 };
uint8_t lora_data[3];

uint16_t userChannelsMask[6]={ 0x00FF,0x0000,0x0000,0x0000,0x0000,0x0000 };

static uint8_t counter=0;

const int n = 10;       // Nombre de valeurs à prendre en compte
float mesures[n];       // Tableau pour stocker les mesures (type float)
int inde = 0;          // Indice actuel dans le tableau


const byte TRIGGER_PIN = GPIO4; // Broche TRIGGER
const byte ECHO_PIN = GPIO5;    // Broche ECHO
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

///////////////////////////////////////////////////
void setup() {
	Serial.begin(115200);


  /* Initialise les broches */
  pinMode(TRIGGER_PIN, OUTPUT);
  digitalWrite(TRIGGER_PIN, LOW); // La broche TRIGGER doit être à LOW au repos
  pinMode(ECHO_PIN, INPUT);
  
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

  while (inde < n) {

  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
  
  long measure = pulseIn(ECHO_PIN, HIGH, MEASURE_TIMEOUT);
  uint16_t dist = measure / 2.0 * SOUND_SPEED *10;
  
  mesures[inde] = dist; // Lecture de la valeur du capteur
  delay(20); // Attendez 20ms avant la prochaine lecture
  inde++; // Incrémentez l'indice
  }

  // Calcul de la médiane et de la moyenne
  float mediane = calculerMedian();
  float moyenneFiltree = calculerMoyenneSansOutliers();

  int distance = mediane;
  // Affichage des valeurs du tableau
  Serial.println("Valeurs du tableau :");
  for (int i = 0; i < n; i++) {
    Serial.print(mesures[i]);
    Serial.print(" ");
  }
  Serial.println();

  // Affichage des résultats
  Serial.print("Mediane: ");
  Serial.println(mediane);
  Serial.print("Moyenne sans outliers: ");
  Serial.println(moyenneFiltree);

  // Réinitialisation pour la prochaine série de mesures
  inde = 0;
  
  lora_data[0] = voltage;
  lora_data[1] = highByte(distance);
  lora_data[2] = lowByte(distance);
  //Now send the data. The parameters are "data size, data pointer, port, request ack"
  Serial.printf("\nSending packet with counter=%d\n", counter);
  //Here we send confirmed packed (ACK requested) only for the first five (remember there is a fair use policy)
  bool requestack=counter<5?true:false;
  if (LoRaWAN.send( sizeof(lora_data), lora_data, 1 , requestack)) {
    Serial.println("Send OK");
  } else {
    Serial.println("Send FAILED");
  }

    //In this demo we use a timer to go into low power mode to kill some time.
  //You might be collecting data or doing something more interesting instead.
  lowPowerSleep(120000); 
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

// Fonction pour calculer la médiane (type float)
float calculerMedian() {
  float valeursTriees[n];
  memcpy(valeursTriees, mesures, sizeof(mesures));
  std::sort(valeursTriees, valeursTriees + n);
  return valeursTriees[n / 2];
}

// Fonction pour calculer la moyenne sans les outliers (type float)
float calculerMoyenneSansOutliers() {
  float mediane = calculerMedian();
  float somme = 0;
  int nombreDeValeurs = 0;
  for (int i = 0; i < n; i++) {
    if (abs(mesures[i] - mediane) <= 2) {
      somme += mesures[i];
      nombreDeValeurs++;
    }
  }
  return somme / (float)nombreDeValeurs;
}
