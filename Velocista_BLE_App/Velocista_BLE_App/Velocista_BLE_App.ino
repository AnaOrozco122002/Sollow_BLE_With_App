// ======================= INCLUDES =======================
#include <Arduino.h>
#include <QTRSensors16.h>
#include <ESP32Servo.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <math.h>   // por fabs()

// ======================= BLE UUIDs =======================
#define SERVICE_UUID          "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID   "beb5483e-36e1-4688-b7f5-ea07361b26a8" // CH1: PID/Vmax/ValTurb (READ/WRITE)
#define CHARACTERISTIC_UUID_2 "ceb5483e-36e1-4688-b7f5-ea07361b26a8" // CH2: Offset/Estado (WRITE) + Lecturas (READ)

BLECharacteristic *pCharacteristic;    // CH1
BLECharacteristic *pCharacteristic_2;  // CH2

// ======================= MÓDULO DE INICIO =======================
const byte MInit = D3;
int   Estado = 0;

// Control remoto del estado (via BLE)
bool  remoteEstadoEnabled = false;
int   remoteEstado = 0; // 0/1

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

// Cache de lecturas para BLE (evita recomputos en READ)
float lastSalidaNorm = 0.0f;  // normalizada [-1, +1]
float lastRawLine    = 0.0f;  // ponderada 0..~15000

// ======================= BLUETOOTH (parse) =======================
String datos;     // buffer para sintonización CH1
String S_offset;  // buffer para offset/estado

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

// ---- Utilidad: recorta espacios/saltos al final ----
static String rtrim(const String& s) {
  int end = s.length() - 1;
  while (end >= 0 && (s[end] == '\r' || s[end] == '\n' || s[end] == ' ' || s[end] == '\t')) end--;
  if (end < 0) return "";
  return s.substring(0, end + 1);
}

// Característica 1: Kp,Ti,Td,Vmax,ValTurb (formato "*kp,ti,td,vmax,valturb")
class MyCallbacks_1 : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pc) override {
    std::string value = pc->getValue();
    if (value.empty()) return;

    // Si el fragmento actual comienza un comando, resetea el buffer
    if (value[0] == '*') {
      datos = "";  // limpia
    }

    // Acumula y normaliza separador decimal
    datos += String(value.c_str());
    datos.replace("-", ".");

    // Soporta '\n' al final, pero no es obligatorio
    int newlineIdx = datos.indexOf('\n');
    if (newlineIdx >= 0) {
      datos = datos.substring(0, newlineIdx);
    }
    datos = rtrim(datos);

    // Se espera "*a,b,c,d,e"
    if (datos.length() < 2 || datos[0] != '*') return;

    // Encuentra comas
    int d1 = datos.indexOf(',', 1);              if (d1 < 0) return;
    int d2 = datos.indexOf(',', d1 + 1);         if (d2 < 0) return;
    int d3 = datos.indexOf(',', d2 + 1);         if (d3 < 0) return;
    int d4 = datos.indexOf(',', d3 + 1);         if (d4 < 0) return;

    // El último campo va hasta el final (no exigimos coma extra)
    String S_Kp      = datos.substring(1, d1);
    String S_Ti      = datos.substring(d1 + 1, d2);
    String S_Td      = datos.substring(d2 + 1, d3);
    String S_Vmax    = datos.substring(d3 + 1, d4);
    String S_ValTurb = datos.substring(d4 + 1);

    datos = ""; // limpia para el próximo

    Kp      = S_Kp.toFloat();
    Ti      = S_Ti.toFloat();
    Td      = S_Td.toFloat();
    Vmax    = constrain(S_Vmax.toFloat(), 0, 1023);
    ValTurb = constrain((int)S_ValTurb.toFloat(), minvaltur, maxvaltur);

    Serial.printf("[BLE CH1] kp=%.3f ti=%.3f td=%.3f vmax=%.0f turb=%d\n",
                  Kp, Ti, Td, Vmax, ValTurb);

    // Actualiza el valor legible de CH1 (snapshot parámetros)
    char pbuf[96];
    snprintf(pbuf, sizeof(pbuf),
             "kp=%.3f,ti=%.3f,td=%.3f,vmax=%.0f,valturb=%d,offset=%.3f",
             (double)Kp, (double)Ti, (double)Td, (double)Vmax, ValTurb, (double)offset);
    pc->setValue((uint8_t*)pbuf, strlen(pbuf));
  }

  void onRead(BLECharacteristic *pc) override {
    // Devuelve snapshot de parámetros al hacer READ
    char pbuf[96];
    snprintf(pbuf, sizeof(pbuf),
             "kp=%.3f,ti=%.3f,td=%.3f,vmax=%.0f,valturb=%d,offset=%.3f",
             (double)Kp, (double)Ti, (double)Td, (double)Vmax, ValTurb, (double)offset);
    pc->setValue((uint8_t*)pbuf, strlen(pbuf));
  }
};

// Característica 2: WRITE para offset/estado; READ para lecturas de sensor
// Formatos aceptados en WRITE:
//   "OFFSET=1.0"  (o solo "1.0")
//   "ESTADO=0"    o "ESTADO=1"
class MyCallbacks_2 : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pc2) override {
    std::string value = pc2->getValue();
    if (value.empty()) return;

    S_offset = String(value.c_str());
    S_offset.replace("-", "."); // compat con clientes raros
    S_offset.trim();            // <-- in-place (NO asignar)

    // Normaliza a mayúsculas para el prefijo (sin tocar números)
    String upper = S_offset;
    upper.toUpperCase();

    if (upper.startsWith("ESTADO=")) {
      int val = upper.substring(7).toInt();
      val = (val != 0) ? 1 : 0;
      remoteEstadoEnabled = true;
      remoteEstado = val;
      Serial.printf("[BLE CH2] ESTADO=%d (remoto habilitado)\n", remoteEstado);
    } else if (upper.startsWith("OFFSET=")) {
      String num = S_offset.substring(7);
      offset = num.toFloat();
      Serial.println("[BLE CH2] OFFSET=" + String(offset));
    } else {
      // Si solo vino un número, tómalo como offset
      bool soloNumero = true;
      for (size_t i = 0; i < S_offset.length(); ++i) {
        char c = S_offset[i];
        if (!((c >= '0' && c <= '9') || c == '.' || c == '+' || c == '-' )) {
          soloNumero = false; break;
        }
      }
      if (soloNumero) {
        offset = S_offset.toFloat();
        Serial.println("[BLE CH2] OFFSET=" + String(offset));
      } else {
        Serial.println("[BLE CH2] Comando no reconocido (use OFFSET=... o ESTADO=0/1).");
      }
    }

    // Refleja offset en snapshot de parámetros (CH1)
    if (pCharacteristic) {
      char pbuf[96];
      snprintf(pbuf, sizeof(pbuf),
               "kp=%.3f,ti=%.3f,td=%.3f,vmax=%.0f,valturb=%d,offset=%.3f",
               (double)Kp, (double)Ti, (double)Td, (double)Vmax, ValTurb, (double)offset);
      pCharacteristic->setValue((uint8_t*)pbuf, strlen(pbuf));
    }
  }

  void onRead(BLECharacteristic *pc2) override {
    // Devuelve las lecturas cacheadas para ahorrar CPU
    char sbuf[64];
    snprintf(sbuf, sizeof(sbuf), "salida=%.3f,raw=%.0f",
             (double)lastSalidaNorm, (double)lastRawLine);
    pc2->setValue((uint8_t*)sbuf, strlen(sbuf));
  }
};

// ======================= PROTOTIPOS =======================
void Inicializacion_Pines();
void Inicializacion_turbina();
void Inicializacion_Sensores();
void CrearPWM();
void ActualizarLecturasCache();
float Lectura_Sensor();
float Controlador(float Referencia, float Salida);
void  Esfuerzo_Control(float Control);
void  Esfuerzo_Turbina();
unsigned long Tiempo_Muestreo(unsigned long Tinicio);
void Inicializacion_Bluetooth();

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
  // Si remoto habilitado, usamos remoteEstado; si no, leemos el pin
  int estadoHW = digitalRead(MInit);
  Estado = remoteEstadoEnabled ? remoteEstado : estadoHW;

  while (Estado) {
    // Refresca Estado en cada iteración
    int estadoLoop = remoteEstadoEnabled ? remoteEstado : digitalRead(MInit);
    Estado = estadoLoop;

    Tinicio  = millis();

    Salida   = Lectura_Sensor();       // actualiza lastRawLine/lastSalidaNorm
    Control  = Controlador(Referencia, Salida);
    Esfuerzo_Control(Control);

    Tm       = Tiempo_Muestreo(Tinicio);

    // Turbina: fija por ValTurb (o usa proporcional si lo deseas)
    myTurbina.write(ValTurb);
    // Esfuerzo_Turbina();

    // Actualiza valor legible de CH2 (sin notificar)
    ActualizarLecturasCache();

    turen = true;
  }

  // Si salimos del while (parado)
  if (turen) {
    ledcWrite(Canales[0], 0);
    ledcWrite(Canales[1], 0);
    myTurbina.write(0);
    ActualizarLecturasCache();
  }

  // Seguridad (motores off)
  ledcWrite(Canales[0], 0);
  ledcWrite(Canales[1], 0);

  // Actualización lenta del valor legible para BLE cuando está parado
  static uint32_t lastIdleUpd = 0;
  uint32_t now = millis();
  if (now - lastIdleUpd > 100) { // 10 Hz en idle
    Lectura_Sensor();
    ActualizarLecturasCache();
    lastIdleUpd = now;
  }
}

// ======================= FUNCIONES =======================

// Lee posición de la línea y actualiza caches
float Lectura_Sensor(void) {
  // ponderada 0..~15000
  float rl = (float)qtra.readLine(sensorValues);
  lastRawLine = rl;

  // normalizada aprox [-1, +1]
  Salida = (rl / 7500.0f) - 1.0f;
  lastSalidaNorm = Salida;

  return Salida;
}

void ActualizarLecturasCache() {
  if (!pCharacteristic_2) return;
  char sbuf[64];
  snprintf(sbuf, sizeof(sbuf), "salida=%.3f,raw=%.0f",
           (double)lastSalidaNorm, (double)lastRawLine);
  pCharacteristic_2->setValue((uint8_t*)sbuf, strlen(sbuf));
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
  // Calibración simple de QTR
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

  // Ampliar MTU para mensajes largos
  BLEDevice::setMTU(185);

  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Característica 1: READ/WRITE (sin notify)
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ   |
    BLECharacteristic::PROPERTY_WRITE
  );
  pCharacteristic->setCallbacks(new MyCallbacks_1());
  // Valor inicial legible
  {
    char pbuf[96];
    snprintf(pbuf, sizeof(pbuf),
             "kp=%.3f,ti=%.3f,td=%.3f,vmax=%.0f,valturb=%d,offset=%.3f",
             (double)Kp, (double)Ti, (double)Td, (double)Vmax, ValTurb, (double)offset);
    pCharacteristic->setValue((uint8_t*)pbuf, strlen(pbuf));
  }

  // Característica 2: WRITE para offset/estado; READ para lecturas
  pCharacteristic_2 = pService->createCharacteristic(
    CHARACTERISTIC_UUID_2,
    BLECharacteristic::PROPERTY_READ   |
    BLECharacteristic::PROPERTY_WRITE
  );
  pCharacteristic_2->setCallbacks(new MyCallbacks_2());
  // Valor inicial de lecturas
  {
    char sbuf[64];
    snprintf(sbuf, sizeof(sbuf), "salida=%.3f,raw=%.0f",
             (double)lastSalidaNorm, (double)lastRawLine);
    pCharacteristic_2->setValue((uint8_t*)sbuf, strlen(sbuf));
  }

  pService->start();

  // Advertising con UUID del servicio
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->start();

  Serial.println("BLE Advertising iniciado (SOLLOW).");
}
