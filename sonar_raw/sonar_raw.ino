/* 
 * Code d'exemple pour un capteur à ultrasons HC-SR04.
 */

/* Constantes pour les broches */
const byte TRIGGER_PIN = GPIO2; // Broche TRIGGER
const byte ECHO_PIN = GPIO3;    // Broche ECHO
const byte TRANS_PIN = GPIO4;    // Broche ECHO

/* Constantes pour le timeout */
const unsigned long MEASURE_TIMEOUT = 25000UL; // 25ms = ~8m à 340m/s

/* Vitesse du son dans l'air en mm/us */
const float SOUND_SPEED = 340.0 / 1000;

/** Fonction setup() */
void setup() {
   
  /* Initialise le port série */
  Serial.begin(115200);
  pinMode(TRANS_PIN, OUTPUT);

  /* Initialise les broches */
  pinMode(TRIGGER_PIN, OUTPUT);
  digitalWrite(TRIGGER_PIN, LOW); // La broche TRIGGER doit être à LOW au repos
  pinMode(ECHO_PIN, INPUT);
}
 
/** Fonction loop() */
void loop() {
  digitalWrite(TRANS_PIN, HIGH);
  delay(1000);
  /* 1. Lance une mesure de distance en envoyant une impulsion HIGH de 10µs sur la broche TRIGGER */
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
  
  /* 2. Mesure le temps entre l'envoi de l'impulsion ultrasonique et son écho (si il existe) */
  long measure = pulseIn(ECHO_PIN, HIGH, MEASURE_TIMEOUT);
   
  /* 3. Calcul la distance à partir du temps mesuré */
  uint16_t distance = measure / 2.0 * SOUND_SPEED;
   
  /* Affiche les résultats en mm, cm et m */
  Serial.print(F("Distance: "));
  Serial.print(distance);
  Serial.println(" mm");
  Serial.println(sizeof(distance));

  digitalWrite(TRANS_PIN, LOW);
  delay(1000); 
  /* Délai d'attente pour éviter d'afficher trop de résultats à la seconde */
  delay(500);
}
