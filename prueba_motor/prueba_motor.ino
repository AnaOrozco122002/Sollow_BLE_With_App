// === IFX + motor coreless test (XIAO ESP32-S3) ===
// Dirección fija en 0 y PWM en D8

#include <Arduino.h>

// Pines (ajusta si tu DIR va a otro pin)
const int PIN_DIR_I = D6;   // si no lo usas por cable, deja este y lo pones LOW por software
const int PIN_PWM_I = D5;   // PWM hacia el IFX

const int PIN_DIR_D = D7;   // si no lo usas por cable, deja este y lo pones LOW por software
const int PIN_PWM_D = D8;   // PWM hacia el IFX

// Config PWM (LEDC)
const int PWM_CHANNEL_D = 0;      // canal LEDC (0..7)
const int PWM_CHANNEL_I = 1;      // canal LEDC (0..7)
const int PWM_FREQ    = 5000;  // 20 kHz recomendado para coreless
const int PWM_RES     = 10;     // bits de resolución (10 bits -> duty 0..1023)

void setup() {
  //Serial.begin(9600);
  // Dirección fija en 0 (si no conectas este pin, puedes puentear DIR a GND en el IFX)
  pinMode(PIN_DIR_D, OUTPUT);
  digitalWrite(PIN_DIR_D, LOW);

  pinMode(PIN_DIR_I, OUTPUT);
  digitalWrite(PIN_DIR_I, LOW);

  // Configura LEDC en ESP32
  /*ledcSetup(PWM_CHANNEL_D, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_PWM_D, PWM_CHANNEL_D);



  // Configura LEDC en ESP32
  ledcSetup(PWM_CHANNEL_I, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_PWM_I, PWM_CHANNEL_I);

  // Arranca motor apagado
  ledcWrite(PWM_CHANNEL_D, 0);
  ledcWrite(PWM_CHANNEL_I, 0);*/

  //Para libreria 3.3.1
  analogWriteResolution(PIN_PWM_D, PWM_RES);
  analogWriteResolution(PIN_PWM_I, PWM_RES);

  analogWriteFrequency(PIN_PWM_D, PWM_FREQ);
  analogWriteFrequency(PIN_PWM_I, PWM_FREQ);


  // Asegura 0 al inicio
  analogWrite(PIN_PWM_D, 0);
  analogWrite(PIN_PWM_I, 0);
}

void loop() {
  // Rampa de aceleración
  for (int duty = 0; duty <= (1 << PWM_RES) - 1; duty += 16) {
    analogWrite(PIN_PWM_D, duty);
    analogWrite(PIN_PWM_I, duty);
    //Serial.println(duty);
    delay(30);
  }

  // Mantén alta velocidad 2 s
  delay(2000);

  // Rampa de desaceleración
  for (int duty = (1 << PWM_RES) - 1; duty >= 0; duty -= 16) {
    analogWrite(PIN_PWM_D, duty);
    analogWrite(PIN_PWM_I, duty);
    //Serial.println(duty);
    delay(30);
  }

  /*
  // Rampa de aceleración
  for (int duty = 0; duty <= (1 << PWM_RES) - 1; duty += 16) {
    ledcWrite(PWM_CHANNEL_D, duty);
    ledcWrite(PWM_CHANNEL_I, duty);
    Serial.println(duty);
    delay(30);
  }

  // Mantén alta velocidad 2 s
  delay(2000);

  // Rampa de desaceleración
  for (int duty = (1 << PWM_RES) - 1; duty >= 0; duty -= 16) {
    ledcWrite(PWM_CHANNEL_D, duty);
    ledcWrite(PWM_CHANNEL_I, duty);
    Serial.println(duty);
    delay(30);
  }*/

  // Pausa con motor apagado
  delay(1500);
}


  //Si quieres una velocidad fija sin rampas, cambia loop() por:

  /*void loop() {
    const int dutyFijo = 500; // entre 0 y 1023 para 10 bits
    ledcWrite(PWM_CHANNEL, dutyFijo);
    // nada más
  }*/

