#include <PID_v1.h>

#define LOG             1                         // Activer les LOGs
#define INTERRUPT_PIN   2                         // Pin d'interruption frequencielle (50Hz)
#define ENGINE_IDLE     155                       // Ralentis (valeur minimum d'accelération)
#define ENGINE_MAX      250                       // Max d'acc
#define ADC_RESOLUTION  8                         // ADC 8 bit
#define PID_FREQUENCY   50                        // Période correspondante à 50Hz (en microsecondes)
#define COMPUTE_PERIOD  1000000 / PID_FREQUENCY   // Période correspondante à la frequence de calcul (en microsecondes)
#define PROTECT_RANGE   6                         // Coupure en cas de problème +-6Hz
#define PROTECT_PERIOD  5                         // Coupure si problème sur + 10 periodes
#define SECURITY_RELAY  3                         // Pin du relais de sécurité (coupure électrique)
#define INPUT_FILTER    10                        // Filtre mauvaise valeurs

// Moteur tourne ou pas
bool engine = true;

// Variables connectées au PID
double input = 0, output = 0;

// Définition PID & paramètres
double kp = 2, ki = 0.7, kd = 0.2, command = PID_FREQUENCY;
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
  digitalWrite(SECURITY_RELAY, HIGH);

  // Interuption 50Hz (sortie comparateur)
  pinMode(INTERRUPT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), interrupt, RISING);

  // Initialisation du PID
  motor_pid.SetMode(AUTOMATIC);
  motor_pid.SetOutputLimits(ENGINE_IDLE, ENGINE_MAX);
}

void loop() {
  unsigned long loop_start = micros();
  if (loop_start >= last_interrupt + COMPUTE_PERIOD * 10 || last_interrupt == 0) { // Pas de signal depuis un certain temps (ou jamais reçu)
    control(ENGINE_IDLE);

    if (engine) {
      engine = false;
      #ifdef LOG
        Serial.println("Engine is OFF.");
      #endif
    }
  } else { // Il y a du signal, calcul
    if (!engine) {
      engine = true;
    }
    
    motor_pid.Compute();
    
    control((int) output);
  }

  long loop_wait = COMPUTE_PERIOD - (micros() - loop_start);
  if (loop_wait <= 0) { // Overload système, la boucle prend trop de temps à s'exécuter
    Serial.println("System is overloaded !");
    return;
  }
  delayMicroseconds(loop_wait);
}

unsigned short protect_count = PROTECT_PERIOD + 1;
unsigned short period_count = 0;

void interrupt() {
  unsigned long actual_interrupt = micros();

  if (period_count <= 2) {
    period_count++;
    return;
  } else {
    period_count = 0;
  }

  if (last_interrupt > 0) { // Ignorer la première interruption
    unsigned long difference = actual_interrupt - last_interrupt;

    double frequency = (double) (200000000 / difference) / 100.0;

    if (frequency > PID_FREQUENCY - INPUT_FILTER
          &&
            (frequency > input + INPUT_FILTER
          || frequency < input - INPUT_FILTER)) {
      #ifdef LOG
        Serial.print("Filtering bad value : ");
        Serial.print(frequency);
        Serial.println("Hz");
      #endif
    } else {
      input = frequency;
      
      #ifdef LOG
        Serial.print("Power = ");
        Serial.print(map((int) output, ENGINE_IDLE, ENGINE_MAX, 0, 100));
        Serial.print("%, Freq = ");
        Serial.print(input);
        Serial.println("Hz");
      #endif
    }
    
    // Protection
    if (input >= PID_FREQUENCY + PROTECT_RANGE
          || input <= PID_FREQUENCY - PROTECT_RANGE) { // Problème de régulation
      if (protect_count > PROTECT_PERIOD) {
        digitalWrite(SECURITY_RELAY, HIGH);
        #ifdef LOG
          Serial.println("Security relay active ! (frequency not in tolerance)");
        #endif
      } else {
        protect_count++;
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
          Serial.println("Security relay released.");
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
