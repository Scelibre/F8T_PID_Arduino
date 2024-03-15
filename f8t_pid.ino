#include <PID_v1.h>

#define LOG             1           // Activer les LOGs
#define INTERRUPT_PIN   2           // Pin d'interruption frequencielle (50Hz)
#define ENGINE_IDLE     100         // Ralentis (valeur minimum d'accelération)
#define ADC_RESOLUTION  8           // ADC 8 bit
#define PID_PERIOD      1000000 / 50   // Période correspondante à 50Hz (en microsecondes)
#define COMPUTE_PERIOD  PID_PERIOD  // Période correspondante à la frequence de calcul (en microsecondes)
#define PROTECT_RANGE   1000000 / 6    // Coupure en cas de problème +-6Hz
#define PROTECT_PERIOD  10          // Coupure si problème sur + 10 periodes
#define SECURITY_RELAY  4           // Pin du relais de sécurité (coupure électrique)

// Variables connectées au PID
double input = 0, output = 0;

// Définition PID & paramètres
double kp = 2, ki = 5, kd = 1, command = PID_PERIOD;
PID motor_pid(&input, &output, &command, kp, ki, kd, DIRECT);

// Enregistrement de la dernière interruption
unsigned long last_interrupt = 0;

void setup() {
  #ifdef LOG
    // Moniteur
    Serial.begin(115200);
  #endif
  
  // Sortie contrôle d'accelération
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);

  // Sortie relais de sécurité
  pinMode(SECURITY_RELAY, OUTPUT);

  // Interuption 50Hz (sortie comparateur)
  pinMode(INTERRUPT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), interrupt, RISING);

  // Initialisation du PID
  motor_pid.SetMode(AUTOMATIC);
  motor_pid.SetOutputLimits(ENGINE_IDLE, pow(2, ADC_RESOLUTION) - 1);

  // Pause pour éviter le démarrage prématuré du calcul.
  delay(PID_PERIOD * 10);
}

void loop() {
  unsigned long loop_start = micros();
  if (loop_start >= last_interrupt + PID_PERIOD * 10) { // Pas de signal depuis un certain temps (ou jamais reçu)
    control(ENGINE_IDLE);
          
    #ifdef LOG
      Serial.println("Engine is OFF.");
    #endif
  } else { // Il y a du signal, calcul
    motor_pid.Compute();
    
    control((int) output);

    #ifdef LOG
      Serial.print("Power = ");
      Serial.print(map((int) output, ENGINE_IDLE, pow(2, ADC_RESOLUTION) - 1, 0, 100));
      Serial.print("%, Freq = ");
      Serial.print((float) (10000000 / input) / 10.0);
      Serial.println("Hz");
    #endif
  }

  long loop_wait = COMPUTE_PERIOD - (micros() - loop_start);
  if (loop_wait <= 0) { // Overload système, la boucle prend trop de temps à s'exécuter
    Serial.println("System is overloaded !");
    return;
  }
  delayMicroseconds(loop_wait);
}

unsigned short protect_count = PROTECT_PERIOD + 1;

void interrupt() { 
  unsigned long actual_interrupt = micros();
  
  if (last_interrupt > 0) { // Ignorer la première interruption
    unsigned long difference = actual_interrupt - last_interrupt;
    input = (double) difference;

    // Protection
    if (difference >= PID_PERIOD + PROTECT_RANGE
          || difference <= PID_PERIOD - PROTECT_RANGE) { // Problème de régulation
      protect_count++;
      if (protect_count > PROTECT_PERIOD) {
        digitalWrite(SECURITY_RELAY, HIGH);
        #ifdef LOG
          Serial.print("Security relay active ! (frequency not in tolerance)");
        #endif
      } else {
        #ifdef LOG
          Serial.print("Warning, frequency not in tolerance (");
          Serial.print(protect_count);
          Serial.print("/");
          Serial.print(PROTECT_PERIOD);
          Serial.println(")");
        #endif
      }
    } else {
      if (protect_count > PROTECT_PERIOD) {
        digitalWrite(SECURITY_RELAY, LOW);
        #ifdef LOG
          Serial.print("Security relay released.");
        #endif
      }
      protect_count = 0;
    }
  }
  
  last_interrupt = actual_interrupt;
}

void control(int value) { // Contrôle accèlération Renault 1.9dCi F8T
  analogWrite(5, value);
  analogWrite(6, value / 2);
}
