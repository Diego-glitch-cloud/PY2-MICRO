// ===================================================================
// AXIS MOTORS - Sistema de Carro Inteligente IoT
// Grupo 6: Diego C. S., Pedro C. T., Hugo M. L., Arodi C. R.
// Programacion de Microprocesadores - UVG
// ===================================================================

#include <Wire.h>
#include <ESP8266WiFi.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

// -------------------------------
// CONFIGURACIÓN WIFI
// -------------------------------
const char* WIFI_SSID = "CLARO1_F4A959";
const char* WIFI_PASS = "21938DEKzc";

// URL de Google Script
String GOOGLE_SCRIPT_URL =
  "https://script.google.com/macros/s/AKfycbxMLoIvZ_iDyRddpPzAai2Jp2L7MWzpDG7zfOlQCU4mtWZyF1gqvWojBjc1WNohcj0/exec";

// Intervalo de envío
#define SHEETS_INTERVAL 10000
unsigned long lastSheetsMillis = 0;

// ===================================================================
// DEFINICIÓN DE PINES
// ===================================================================

// BMP280
#define BMP_SDA 4     // D2 - GPIO4
#define BMP_SCL 5     // D1 - GPIO5

// Microfono KY-037/038
#define MIC_ANALOG A0

// Fotorresistencia (digital)
#define LUZ_DO 16

// HC-SR04
#define TRIG_PIN 14
#define ECHO_PIN 13

// Sensor táctil
#define TOUCH_PIN 12

// Actuadores
#define MOTOR_PIN 0
#define BUZZER_PIN 15
#define FARO_PIN 2

// ===================================================================
// ESTADOS
// ===================================================================
#define APAGADO 0
#define ENCENDIDO 1

// Distancia
#define DIST_ALARMA 30
#define DIST_CRITICA 10

#define LUZ_INTERVAL 500
#define DIST_INTERVAL 200
#define MIC_INTERVAL 300

// ===================================================================
// VARIABLES GLOBALES
// ===================================================================
int car_state = APAGADO;

Adafruit_BMP280 bmp;

unsigned long lastLuzMillis = 0;
unsigned long lastDistMillis = 0;
unsigned long lastMicMillis = 0;

bool prev_touch_state = false;
bool faros_encendidos = false;

bool luz_oscura = false;
float distancia_actual = 0.0;
int nivel_ruido = 0;

float temperatura_actual = 0.0;
float presion_hpa = 0.0;

// ===================================================================
// SETUP
// ===================================================================

void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println("\n===================================");
  Serial.println("AXIS MOTORS - Sistema Iniciando...");
  Serial.println("===================================\n");

  pinMode(TOUCH_PIN, INPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LUZ_DO, INPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(FARO_PIN, OUTPUT);

  digitalWrite(MOTOR_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(FARO_PIN, LOW);

  // ------------------------------------
  // BMP280 
  // ------------------------------------
  Wire.begin(BMP_SDA, BMP_SCL);
  delay(80);

  if (!bmp.begin(0x76)) {
    Serial.println("BMP280 no detectado (0x76). Probando 0x77...");
    if (!bmp.begin(0x77)) {
      Serial.println("BMP280 NO FUNCIONA. Se enviarán ceros.");
    } else {
      Serial.println("BMP280 detectado en 0x77");
    }
  } else {
    Serial.println("BMP280 detectado en 0x76");
  }

  // ------------------------------------
  // Inicio de WiFi
  // ------------------------------------
  Serial.println("\nConectando a WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  int intentos = 0;
  while (WiFi.status() != WL_CONNECTED && intentos < 20) {
    delay(500);
    Serial.print(".");
    intentos++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi conectado.");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nNO se pudo conectar (offline mode).");
  }

  Serial.println("\nSistema listo.");
  Serial.println("ESTADO: APAGADO");
}

// ===================================================================
// LOOP
// ===================================================================

void loop() {
  unsigned long now = millis();

  gestionarEncendidoTactil();

  if (now - lastLuzMillis >= LUZ_INTERVAL) {
    lastLuzMillis = now;
    leerFotorresistencia();

    if (car_state == ENCENDIDO) controlarFaros();
    else apagarFaros();
  }

  if (now - lastDistMillis >= DIST_INTERVAL) {
    lastDistMillis = now;
    asistenciaRetroceso();
  }

  if (now - lastMicMillis >= MIC_INTERVAL) {
    lastMicMillis = now;
    if (car_state == ENCENDIDO) leerNivelRuido();
  }

  actualizarMotor();

  if (now - lastSheetsMillis >= SHEETS_INTERVAL) {
    lastSheetsMillis = now;
    enviarDatosSheets();
  }
}

// ===================================================================
// FUNCIONES
// ===================================================================

void gestionarEncendidoTactil() {
  bool touch_state = digitalRead(TOUCH_PIN);

  if (touch_state == HIGH && prev_touch_state == LOW) {
    delay(50);

    if (car_state == APAGADO) {
      car_state = ENCENDIDO;
      Serial.println("\n=== MOTOR ENCENDIDO ===\n");
    } else {
      car_state = APAGADO;
      Serial.println("\n=== MOTOR APAGADO ===\n");
    }
  }

  prev_touch_state = touch_state;
}

void leerFotorresistencia() {
  luz_oscura = (digitalRead(LUZ_DO) == HIGH);
}

void controlarFaros() {
  if (luz_oscura && !faros_encendidos) {
    digitalWrite(FARO_PIN, HIGH);
    faros_encendidos = true;
  } else if (!luz_oscura && faros_encendidos) {
    apagarFaros();
  }
}

void apagarFaros() {
  digitalWrite(FARO_PIN, LOW);
  faros_encendidos = false;
}

void asistenciaRetroceso() {
  distancia_actual = medirDistancia();

  if (distancia_actual <= 0) {
    digitalWrite(BUZZER_PIN, LOW);
    return;
  }

  if (car_state == ENCENDIDO) {
    if (distancia_actual <= DIST_CRITICA) digitalWrite(BUZZER_PIN, HIGH);
    else if (distancia_actual <= DIST_ALARMA) {
      static unsigned long lastBeep = 0;
      if (millis() - lastBeep > 300) {
        digitalWrite(BUZZER_PIN, !digitalRead(BUZZER_PIN));
        lastBeep = millis();
      }
    } else digitalWrite(BUZZER_PIN, LOW);
  } else digitalWrite(BUZZER_PIN, LOW);
}

void leerNivelRuido() {
  int suma = 0;
  for (int i = 0; i < 10; i++) {
    suma += analogRead(MIC_ANALOG);
    delayMicroseconds(100);
  }
  nivel_ruido = suma / 10;
}

void actualizarMotor() {
  digitalWrite(MOTOR_PIN, car_state == ENCENDIDO ? HIGH : LOW);
}

float medirDistancia() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duracion = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duracion == 0) return -1;

  return duracion * 0.0343 / 2.0;
}

// -------------------------------
// ENVIAR DATOS A GOOGLE SHEETS
// -------------------------------
void enviarDatosSheets() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Sheets: WiFi NO conectado.");
    return;
  }

  // Leer BMP280 (aunque falle)
  temperatura_actual = bmp.readTemperature();
  presion_hpa = bmp.readPressure() / 100.0;

  if (isnan(temperatura_actual)) temperatura_actual = 0;
  if (isnan(presion_hpa)) presion_hpa = 0;

  int luminosidad_raw = faros_encendidos ? 1 : 0;

  String url = GOOGLE_SCRIPT_URL +
               "?presion=" + String(presion_hpa, 2) +
               "&temp_motor=" + String(temperatura_actual, 1) +
               "&luz=" + String(luminosidad_raw) +
               "&dist=" + String(distancia_actual, 1) +
               "&estado=" + String(car_state);

  Serial.println("\nEnviando a Google Sheets:");
  Serial.println(url);

  WiFiClient client;

  if (!client.connect("script.google.com", 80)) {
    Serial.println("Error al conectar con Google Script");
    return;
  }

  client.println("GET " + url + " HTTP/1.1");
  client.println("Host: script.google.com");
  client.println("Connection: close");
  client.println();

  while (client.connected() || client.available()) {
    if (client.available()) client.readStringUntil('\n');
  }

  client.stop();
}
