#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <SPI.h>
#include <MFRC522.h>
#include <WiFi.h>
#include <SharpIR.h> 

/*************** CONFIGURATION WI-FI ***************/
const char* ssid     = "NOM_DE_VOTRE_BOX_WIFI"; 
const char* password = "MOT_DE_PASSE_WIFI";     

/*************** CONFIGURATION DES SENS MOTEURS ***************/
const int INVERSER_MOT_G = 1;   
const int INVERSER_MOT_D = -1;  

/*************** IMU (Modifié : Variables globales) *****************/
MPU6050 mpu;
int16_t ax, ay, az;
int16_t gx, gy, gz;
volatile float gyroZ_global = 0.0; // Partagé en toute sécurité avec le Timer

/*************** RFID *****************/
#define SS_PIN 21   
#define RST_PIN 14  
MFRC522 mfrc522(SS_PIN, RST_PIN);

/*************** CAPTEURS D'OBSTACLES GT1081 ***************/
#define GT_DROIT 4   
#define GT_GAUCHE 2  

/*************** CAPTEUR DISTANCE SHARP ***************/
#define IRPin 34
#define model 1080   
SharpIR mySensor(IRPin, model);

/*************** SUIVI DE LIGNE TCRT5000 ***************/
#define CAP_G 32     
#define CAP_C 33     
#define CAP_D 25     

/*************** CONFIGURATION MOTEURS MP6550 ***************/
#define DIR_G1 15   
#define DIR_G2 13   
#define DIR_D1 22   
#define DIR_D2 0    

#define PWM_FREQ 5000
#define PWM_RES 8

#define PWM_MIN 60  // Vitesse minimale pour aider le moteur lent à démarrer
#define PWM_MAX 150 

/*************** ENCODEURS FIT0450 ***************/
#define ENC_G 26    
#define ENC_D 16    

volatile long ticksG = 0;
volatile long ticksD = 0;

#define TICKS_PAR_TOUR 20

/*************** PI & VITESSE ***************/
const float Kp = 0.36;
const float Ki = 10.9;
const float Te = 0.01;

volatile float eG=0,eG_prec=0,uG=0,uG_prec=0;
volatile float eD=0,eD_prec=0,uD=0,uD_prec=0;

volatile float consigne = 3.5; // Vitesse lente configurée

/*************** IMU GAIN ***************/
const float Kimu = 0.002;

/*************** TIMER *****************/
hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

/*************** ENCODEURS ISR ***************/
void IRAM_ATTR ISR_G()){ ticksG++; }
void IRAM_ATTR ISR_D()){ ticksD++; }

/*************** SEUIL *****************/
int appliquer_seuil(int pwm){
  if(pwm == 0) return 0;
  if(pwm > 0 && pwm < PWM_MIN) return PWM_MIN;
  if(pwm > PWM_MAX) return PWM_MAX;
  return pwm;
}

/*************** MOTEURS MP6550 *****************/
void moteurG(int pwm){
  int v = appliquer_seuil(abs(pwm));
  if(pwm > 0){
    ledcWrite(DIR_G1, v);   
    digitalWrite(DIR_G2, LOW); 
  }
  else if(pwm < 0){
    digitalWrite(DIR_G1, LOW); 
    ledcWrite(DIR_G2, v);   
  }
  else{
    digitalWrite(DIR_G1, LOW);
    digitalWrite(DIR_G2, LOW);
  }
}

void moteurD(int pwm){
  int v = appliquer_seuil(abs(pwm));
  if(pwm > 0){
    ledcWrite(DIR_D1, v);   
    digitalWrite(DIR_D2, LOW); 
  }
  else if(pwm < 0){
    digitalWrite(DIR_D1, LOW); 
    ledcWrite(DIR_D2, v);   
  }
  else{
    digitalWrite(DIR_D1, LOW);
    digitalWrite(DIR_D2, LOW);
  }
}

/*************** CALCUL VITESSE ***************/
float calcul_vitesse(long ticks){
  return (2.0 * PI * ticks) / (TICKS_PAR_TOUR * Te);
}

/*************** TIMER ISR (Régulation fluide) ***************/
void IRAM_ATTR onTimer(){
  portENTER_CRITICAL_ISR(&timerMux);

  long tG = ticksG; ticksG = 0;
  long tD = ticksD; ticksD = 0;

  float omegaG = calcul_vitesse(tG);
  float omegaD = calcul_vitesse(tD);

  // Utilisation de la valeur lue proprement en tâche de fond
  float correction = Kimu * gyroZ_global;

  /******** PI ********/
  eG = consigne - omegaG - correction;
  eD = consigne - omegaD + correction;

  uG = uG_prec + Kp*(eG - eG_prec) + Ki*Te*eG;
  uD = uD_prec + Kp*(eD - eD_prec) + Ki*Te*eD;

  // Saturation
  if(uG > PWM_MAX) uG = PWM_MAX;
  if(uG < -PWM_MAX) uG = -PWM_MAX;
  if(uD > PWM_MAX) uD = PWM_MAX;
  if(uD < -PWM_MAX) uD = -PWM_MAX;

  // Commande instantanée et synchrone des deux moteurs
  moteurG(uG * INVERSER_MOT_G);
  moteurD(uD * INVERSER_MOT_D);

  uG_prec = uG; eG_prec = eG;
  uD_prec = uD; eD_prec = eD;

  portEXIT_CRITICAL_ISR(&timerMux);
}

/*************** SETUP ***************/
void setup(){
  Serial.begin(115200);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connecté !");
  Serial.print("➡️ IP du robot : "); Serial.println(WiFi.localIP());

  pinMode(DIR_G1, OUTPUT); pinMode(DIR_G2, OUTPUT);
  pinMode(DIR_D1, OUTPUT); pinMode(DIR_D2, OUTPUT);

  ledcAttach(DIR_G1, PWM_FREQ, PWM_RES);
  ledcAttach(DIR_G2, PWM_FREQ, PWM_RES);
  ledcAttach(DIR_D1, PWM_FREQ, PWM_RES);
  ledcAttach(DIR_D2, PWM_FREQ, PWM_RES);

  pinMode(ENC_G, INPUT_PULLUP);
  pinMode(ENC_D, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_G), ISR_G, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_D), ISR_D, RISING);

  pinMode(GT_GAUCHE, INPUT);
  pinMode(GT_DROIT, INPUT);

  pinMode(CAP_G, INPUT);
  pinMode(CAP_C, INPUT);
  pinMode(CAP_D, INPUT);

  // Initialisation I2C pour le gyroscope
  Wire.begin();
  mpu.initialize();

  SPI.begin();
  mfrc522.PCD_Init();

  // Initialisation du Horloger/Timer (10 ms)
  timer = timerBegin(1000000); 
  timerAttachInterrupt(timer, &onTimer);
  timerAlarm(timer, 10000, true, 0); 
}

/*************** LOOP ***************/
void loop(){
  
  // 1️⃣ LECTURE DU GYROSCOPE (Déplacée ici pour libérer le processeur !)
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  portENTER_CRITICAL(&timerMux);
  gyroZ_global = gz / 131.0; 
  portEXIT_CRITICAL(&timerMux);

  // 2️⃣ LECTURE CAPTEURS D'OBSTACLES GT1081
  bool obsG = (digitalRead(GT_GAUCHE) == LOW);
  bool obsD = (digitalRead(GT_DROIT) == LOW);
  Serial.print("GT1081 -> G: "); Serial.print(obsG); Serial.print(" | D: "); Serial.println(obsD);

  // 3️⃣ LECTURE DISTANCE SHARP
  int distance_cm = mySensor.distance(); 
  Serial.print("Sharp IR -> Distance: "); Serial.print(distance_cm); Serial.println(" cm");

  // 4️⃣ LECTURE SUIVI DE LIGNE TCRT5000
  int ligneG = digitalRead(CAP_G);
  int ligneC = digitalRead(CAP_C);
  int ligneD = digitalRead(CAP_D);
  Serial.print("TCRT5000 -> G: "); Serial.print(ligneG); Serial.print(" | C: "); Serial.print(ligneC); Serial.print(" | D: "); Serial.println(ligneD);

  // 5️⃣ ROUTINE LECTURE BADGES RFID
  if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
    Serial.print("RFID -> UID détecté : ");
    for(byte i=0; i<mfrc522.uid.size; i++){
      Serial.print(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " ");
      Serial.print(mfrc522.uid.uidByte[i], HEX);
    }
    Serial.println();
    mfrc522.PICC_HaltA();
  }

  Serial.println("---------------------------------------");
  delay(300); 
}
