// ======================= INCLUDES =======================
#include <Arduino.h>
#include <QTRSensors16.h>
#include <ESP32Servo.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <math.h>   // por fabs()

// ======================= BLE UUIDs =======================
#define SERVICE_UUID          "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID   "beb5483e-36e1-4688-b7f5-ea07361b26a8" // CH1: sintonización PID/Vmax/ValTurb
#define CHARACTERISTIC_UUID_2 "ceb5483e-36e1-4688-b7f5-ea07361b26a8" // CH2: offset

BLECharacteristic *pCharacteristic;
BLECharacteristic *pCharacteristic_2;

// ======================= MÓDULO DE INICIO =======================
const byte MInit = D3;
int   Estado = 0;

// ======================= TURBINA =======================
Servo myTurbina;
const byte Tur = D4;
int   ValTurb = 150;
int   minvaltur = 50;
int   maxvaltur = 180;
float KTurb = 0.6f; // (si usas Esfuerzo_Turbina)

// ======================= SENSORES QTR =======================
#define NUM_SENSORS 16
#define NUM_SAMPLES_PER_SENSOR 3
#define IN_PIN A2 // PIN de entrada del MUX hacia el micro

// Secuencia 0..15 y select lines del MUX: D9, D10, D0, D1
QTRSensorsMux qtra(
  (unsigned char[]){0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15},
  NUM_SENSORS,
  NUM_SAMPLES_PER_SENSOR,
  (unsigned char)IN_PIN,
  (unsigned char[]){ D9, D10, D0, D1 }
);
unsigned int sensorValues[NUM_SENSORS];

// ======================= CONTROL =======================
float Tm = 9.0f; // ms (medido por loop)
float Referencia = 0.0f, Control = 0.0f, Kp = 2.0f, Ti = 0.0f, Td = 0.02f;
float Salida = 0.0f, Error = 0.0f, Error_ant = 0.0f;
float offset = 1.0f, Vmax = 1023.0f, E_integral = 0.0f;

// ======================= BLUETOOTH (parse) =======================
String datos; // buffer para sintonización CH1
String S_offset;

// ======================= PWM Motores =======================
const uint16_t Frecuencia = 5000;
const byte     Canales[]  = { 0, 1 };
const byte     Resolucion = 10;

const int PWMI = D6; // Motor Izquierdo PWM
const int PWMD = D8; // Motor Derecho  PWM
const int DirI = D5; // Dirección Izquierdo
const int DirD = D7; // Dirección Derecho

// ======================= ESTADO RUNTIME =======================
unsigned long Tinicio = 0;
bool conect = false;
bool turen  = false;

// Anti-flood notificaciones BLE
const uint32_t NOTIFY_INTERVAL_MS = 20; // notificar cada 20 ms (50 Hz)
uint32_t lastNotifyMs = 0;

// ======================= BLE CALLBACKS =======================
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    conect = true;
    Serial.println("Cliente conectado.");
  }
  void onDisconnect(BLEServer* pServer) override {
    conect = false;
    Serial.println("Cliente desconectado.");
    pServer->getAdvertising()->start(); // Reinicia advertising
    Serial.println("Publicidad BLE reiniciada.");
  }
};

// Característica 1: Kp,Ti,Td,Vmax,ValTurb (formato "*kp,ti,td,vmax,valturb")
class MyCallbacks_1 : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override {
    std::string value = pCharacteristic->getValue();
    if (value.empty()) return;

    // Acumulamos en buffer por si llega fragmentado
    datos += String(value.c_str());

    // Si el paquete empieza con '*', intentamos parsear completo
    if (value[0] == '*') {
      // Compat: el viejo cliente mandaba '-' para decimal; aquí lo normalizamos
      datos.replace("-", ".");

      int d1 = datos.indexOf(',');
      if (d1 < 0) return;
      String S_Kp = datos.substring(1, d1);

      int d2 = datos.indexOf(',', d1 + 1);
      if (d2 < 0) return;
      String S_Ti = datos.substring(d1 + 1, d2);

      int d3 = datos.indexOf(',', d2 + 1);
      if (d3 < 0) return;
      String S_Td = datos.substring(d2 + 1, d3);

      int d4 = datos.indexOf(',', d3 + 1);
      if (d4 < 0) return;
      String S_Vmax = datos.substring(d3 + 1, d4);

      int d5 = datos.indexOf(',', d4 + 1);
      if (d5 < 0) return;
      String S_ValTurb = datos.substring(d4 + 1, d5);

      // Limpiamos buffer para siguiente comando
      datos = "";

      // Convertimos
      Kp      = S_Kp.toFloat();
      Ti      = S_Ti.toFloat();
      Td      = S_Td.toFloat();
      Vmax    = S_Vmax.toFloat();
      ValTurb = (int)S_ValTurb.toFloat();

      // Clamps mínimos razonables
      if (Vmax < 0) Vmax = 0;
      if (Vmax > 1023) Vmax = 1023;
      ValTurb = constrain(ValTurb, minvaltur, maxvaltur);

      Serial.printf("[BLE CH1] kp=%.3f ti=%.3f td=%.3f vmax=%.0f turb=%d\n",
                    Kp, Ti, Td, Vmax, ValTurb);
    }
  }
};

// Característica 2: offset (texto simple, p.ej. "1.0")
class MyCallbacks_2 : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic_2) override {
    std::string value = pCharacteristic_2->getValue();
    if (value.empty()) return;

    S_offset = String(value.c_str());
    S_offset.replace("-", "."); // por si algún cliente usa '-'
    offset = S_offset.toFloat();
    Serial.println("offset: " + String(offset));
  }
};

// ======================= PROTOTIPOS =======================
void Inicializacion_Pines();
void Inicializacion_turbina();
void Inicializacion_Sensores();
void CrearPWM();
float Lectura_Sensor();
float Controlador(float Referencia, float Salida);
void  Esfuerzo_Control(float Control);
void  Esfuerzo_Turbina();
unsigned long Tiempo_Muestreo(unsigned long Tinicio);
void Inicializacion_Bluetooth();
void EnviarDatos();

// ======================= SETUP =======================
void setup() {
  Serial.begin(115200);

  Inicializacion_Pines();
  Inicializacion_turbina();
  Inicializacion_Sensores();
  CrearPWM();
  Inicializacion_Bluetooth();

  delay(1000);
  myTurbina.write(ValTurb); // posición inicial
}

// ======================= LOOP =======================
void loop() {
  Estado = digitalRead(MInit);
  // Estado = 1; // <-- descomenta para desactivar botón de inicio

  while (Estado) {
    Estado   = digitalRead(MInit);
    Tinicio  = millis();

    Salida   = Lectura_Sensor();
    Control  = Controlador(Referencia, Salida);
    Esfuerzo_Control(Control);

    Tm       = Tiempo_Muestreo(Tinicio);

    // Turbina fija segun ValTurb (o usa Esfuerzo_Turbina() si quieres variable)
    myTurbina.write(ValTurb);
    // Esfuerzo_Turbina(); // Turbina variable basada en Error

    EnviarDatos();
    turen = true;
  }

  // Si salimos del while (parado)
  if (turen) {
    ledcWrite(Canales[0], 0);
    ledcWrite(Canales[1], 0);
    myTurbina.write(0);
    EnviarDatos();
  }

  // Seguridad (motores off)
  ledcWrite(Canales[0], 0);
  ledcWrite(Canales[1], 0);
  EnviarDatos();
}

// ======================= FUNCIONES =======================

// Lee posición de la línea (normalizada aprox. [-1, +1])
float Lectura_Sensor(void) {
  // QTR readLine devuelve ~0..15000; centramos en 0
  Salida = (qtra.readLine(sensorValues) / 7500.0f) - 1.0f;
  return Salida;
}

// Controlador PID (forma estándar con anti-windup y zona muerta pequeña)
float Controlador(float Ref, float Y) {
  float E_derivativo;
  float U;

  Error = Ref - Y;

  // Zona muerta +/-0.2
  if ((Error > -0.2f && Error < 0.0f) || (Error > 0.0f && Error < 0.2f)) {
    Error = 0.0f;
  }

  // Integración trapezoidal
  float Ts = Tm / 1000.0f; // seg
  E_integral += ((Error * Ts) + (Ts * (Error - Error_ant)) / 2.0f);
  E_integral = constrain(E_integral, -100.0f, 100.0f);

  // Derivada
  E_derivativo = (Error - Error_ant) / max(Ts, 1e-3f);

  U = Kp * (Error + Ti * E_integral + Td * E_derivativo);
  Error_ant = Error;

  // Saturación salida control
  U = constrain(U, -2.5f, 2.5f);
  return U;
}

// Envía PWM a motores según offset +/- Control
void Esfuerzo_Control(float U) {
  float s1 = (offset - U); // Derecho
  float s2 = (offset + U); // Izquierdo

  // Magnitud PWM
  int pwm1 = floor(constrain(fabs(s1), 0.0f, 1.0f) * Vmax);
  int pwm2 = floor(constrain(fabs(s2), 0.0f, 1.0f) * Vmax);

  ledcWrite(Canales[0], pwm1);
  ledcWrite(Canales[1], pwm2);

  // Direcciones
  digitalWrite(DirD, (s1 <= 0.0f) ? LOW  : HIGH);
  digitalWrite(DirI, (s2 <= 0.0f) ? LOW  : HIGH);
}

// Turbina proporcional al |Error| (opcional)
void Esfuerzo_Turbina(){
  float estur = constrain(round(minvaltur + ((KTurb * fabs(Error)) * (maxvaltur - minvaltur))),
                          (float)minvaltur, (float)maxvaltur);
  myTurbina.write((int)estur);
}

unsigned long Tiempo_Muestreo(unsigned long TinicioRef) {
  return millis() - TinicioRef;
}

void CrearPWM() {
  ledcSetup(Canales[0], Frecuencia, Resolucion);
  ledcSetup(Canales[1], Frecuencia, Resolucion);
  ledcAttachPin(PWMD, Canales[0]); // CH0 -> Motor Derecho
  ledcAttachPin(PWMI, Canales[1]); // CH1 -> Motor Izquierdo
}

void Inicializacion_turbina() {
  ESP32PWM::allocateTimer(2);
  myTurbina.setPeriodHertz(50);
  myTurbina.attach(Tur, 1000, 2000);
  myTurbina.write(0);
}

void Inicializacion_Sensores() {
  // Calibración simple de QTR (gira manual/levemente el robot sobre la línea durante este tiempo)
  for (int i = 0; i < 300; i++) {
    qtra.calibrate(); // ~25ms por llamada
  }
}

void Inicializacion_Pines() {
  pinMode(PWMD, OUTPUT);
  pinMode(PWMI, OUTPUT);
  pinMode(DirI, OUTPUT);
  pinMode(DirD, OUTPUT);
  pinMode(MInit, INPUT);

  // Estados iniciales seguros
  digitalWrite(DirI, LOW);
  digitalWrite(DirD, LOW);
  ledcWrite(Canales[0], 0);
  ledcWrite(Canales[1], 0);
}

void Inicializacion_Bluetooth() {
  BLEDevice::init("SOLLOW");

  // Ampliar MTU para mensajes más largos (opcional, pero útil)
  BLEDevice::setMTU(185);

  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Característica 1
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ   |
    BLECharacteristic::PROPERTY_WRITE  |
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setCallbacks(new MyCallbacks_1());
  pCharacteristic->setValue("Inicializacion Sollow");

  // Característica 2
  pCharacteristic_2 = pService->createCharacteristic(
    CHARACTERISTIC_UUID_2,
    BLECharacteristic::PROPERTY_READ   |
    BLECharacteristic::PROPERTY_WRITE  |
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pCharacteristic_2->addDescriptor(new BLE2902());
  pCharacteristic_2->setCallbacks(new MyCallbacks_2());
  pCharacteristic_2->setValue("Caracteristica 2");

  pService->start();

  // Advertising con UUID del servicio (para que Android lo descubra fácil)
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->start();

  Serial.println("BLE Advertising iniciado (SOLLOW).");
}

// Envía notificaciones a la app con anti-flood
void EnviarDatos() {
  if (!conect) return;

  uint32_t now = millis();
  if (now - lastNotifyMs < NOTIFY_INTERVAL_MS) return; // limita frecuencia
  lastNotifyMs = now;

  // CH1 -> Telemetría básica CSV: "salida,control,error,tm"
  // Ej. "0.123,-0.456,0.123,9"
  char buf1[64];
  snprintf(buf1, sizeof(buf1), "%.3f,%.3f,%.3f,%.0f", (double)Salida, (double)Control, (double)Error, (double)Tm);
  pCharacteristic->setValue((uint8_t*)buf1, strlen(buf1));
  pCharacteristic->notify();

  // CH2 -> Offset actual
  char buf2[32];
  snprintf(buf2, sizeof(buf2), "offset=%.3f", (double)offset);
  pCharacteristic_2->setValue((uint8_t*)buf2, strlen(buf2));
  pCharacteristic_2->notify();
}
