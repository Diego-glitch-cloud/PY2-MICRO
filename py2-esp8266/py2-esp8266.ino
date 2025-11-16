// ===================================================================
// AXIS MOTORS - Sistema de Carro Inteligente IoT
// Grupo 6: Diego C. S., Pedro C. T., Hugo M. L., Arodi C. R.
// Programación de Microprocesadores - UVG
// ===================================================================

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <MFRC522.h>

// ===================================================================
// GOOGLE SHEETS - CONFIGURACIÓN WIFI Y SCRIPT
// ===================================================================
#include <ESP8266WiFi.h>

const char* WIFI_SSID = "WIFI";
const char* WIFI_PASS = "PASSWORD";

// URL del Apps Script
String GOOGLE_SCRIPT_URL = "https://script.google.com/macros/s/AKfycbxMLoIvZ_iDyRddpPzAai2Jp2L7MWzpDG7zfOlQCU4mtWZyF1gqvWojBjc1WNohcj0/exec";

// Intervalo de envío (ms)
#define SHEETS_INTERVAL 10000
unsigned long lastSheetsMillis = 0;

// ===================================================================
// DEFINICIÓN DE PINES
// ===================================================================

// Sensor BMP280 (I2C)
#define BMP_SDA 4         // D2 - GPIO4
#define BMP_SCL 5         // D1 - GPIO5

// RFID RC522 (SPI)
#define RFID_SS 15        // D8 - GPIO15
#define RFID_RST 0        // D3 - GPIO0
#define RFID_SCK 14       // D5 - GPIO14
#define RFID_MOSI 13      // D7 - GPIO13
#define RFID_MISO 12      // D6 - GPIO12

// Sensor Táctil TTP223
#define TOUCH_PIN 2       // D4 - GPIO2

// Sensor Ultrasónico HC-SR04
#define TRIG_PIN 16       // D0 - GPIO16
#define ECHO_PIN 3        // RX - GPIO3

// Actuadores
#define MOTOR_PIN 1       // TX - GPIO1 (Control Digital ON/OFF)
#define BUZZER_PIN 9      // SD2 - GPIO9 - BUZZER ACTIVO
#define LED_VERDE 14      // D5 - GPIO14 (COMPARTIDO con RFID SCK)
#define FARO_PIN 10       // SD3 - GPIO10

// Sensor de Luz (Fotoresistencia)
#define LUZ_PIN A0        // A0 - ADC

// ===================================================================
// CONSTANTES DEL SISTEMA
// ===================================================================

// Estados del carro
#define BLOQUEADO 0       // Tarjeta no validada - Sistema bloqueado
#define DESBLOQUEADO 1    // Tarjeta aceptada - Motor apagado
#define ENCENDIDO 2       // Tarjeta aceptada - Motor encendido

// Valores PWM calculados (solo para variable interna, no para motor físico)
#define PWM_OFF 0
#define PWM_BAJO 205      // 20%
#define PWM_MEDIO 614     // 60%
#define PWM_ALTO 1023     // 100%

// Umbrales de temperatura (°C)
#define TEMP_OFF 30.0
#define TEMP_BAJO 35.0
#define TEMP_MEDIO 40.0

// Umbrales de luminosidad
#define LUZ_OSCURO 300
#define LUZ_CLARO 400

// Umbrales de distancia (cm)
#define DIST_ALARMA 30
#define DIST_CRITICA 10

// Tiempos de actualización (ms)
#define TEMP_INTERVAL 2000
#define LUZ_INTERVAL 500
#define DIST_INTERVAL 200

// UID válido de ejemplo para RFID (4 bytes)
byte UID_VALIDO[] = {0xDE, 0xAD, 0xBE, 0xEF};

// ===================================================================
// VARIABLES GLOBALES
// ===================================================================

// Estado del sistema (CRÍTICO)
int car_state = BLOQUEADO;

// Objetos de sensores
Adafruit_BMP280 bmp;
MFRC522 rfid(RFID_SS, RFID_RST);

// Control de tiempo
unsigned long lastTempMillis = 0;
unsigned long lastLuzMillis = 0;
unsigned long lastDistMillis = 0;

// Estados previos para detección de flancos
bool prev_touch_state = false;
bool faros_encendidos = false;

// Lecturas de sensores
float temperatura_actual = 0.0;
float presion_hpa = 0.0;
int luz_actual = 0;
float distancia_actual = 0.0;

// Variable PWM calculada (para futuro envío a Blynk)
int pwm_ventilador_calculado = PWM_OFF;

// ===================================================================
// CONFIGURACIÓN INICIAL
// ===================================================================

void setup() {
  Serial.begin(115200);
  Serial.println("\n===================================");
  Serial.println("AXIS MOTORS - Sistema Iniciando...");
  Serial.println("===================================\n");
  
  // Configurar pines de entrada
  pinMode(TOUCH_PIN, INPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LUZ_PIN, INPUT);
  
  // Configurar pines de salida
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(FARO_PIN, OUTPUT);
  pinMode(LED_VERDE, OUTPUT);
  
  // Estado inicial: BLOQUEADO - Todo apagado
  digitalWrite(MOTOR_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(FARO_PIN, LOW);
  digitalWrite(LED_VERDE, LOW);
  
  // Inicializar I2C para BMP280
  Wire.begin(BMP_SDA, BMP_SCL);
  
  if (!bmp.begin(0x76)) {
    Serial.println("ADVERTENCIA: BMP280 no detectado en 0x76, probando 0x77...");
    if (!bmp.begin(0x77)) {
      Serial.println("ERROR: BMP280 no detectado. Revisar conexiones.");
    } else {
      Serial.println("BMP280 inicializado en 0x77");
    }
  } else {
    Serial.println("BMP280 inicializado en 0x76");
  }
  
  // Inicializar SPI para RFID
  SPI.begin();
  rfid.PCD_Init();
  Serial.println("RFID RC522 inicializado");
  Serial.print("UID válido configurado: ");
  printUID(UID_VALIDO, 4);
  
  Serial.println("\nSistema listo.");
  Serial.println("ESTADO ACTUAL: BLOQUEADO");
  Serial.println("Esperando tarjeta RFID valida para desbloquear...\n");

  // Inicializar WIFI
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
    Serial.print("IP asignada: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nERROR: No se pudo conectar a WiFi (modo offline).");
  }

}

// ===================================================================
// BUCLE PRINCIPAL
// ===================================================================

void loop() {
  unsigned long now = millis();
  
  // 1. GESTIÓN DE ACCESO (SIEMPRE ACTIVO)
  gestionarAccesoRFID();
  
  // 2. GESTIÓN DE ENCENDIDO CON TÁCTIL (solo si DESBLOQUEADO o ENCENDIDO)
  if (car_state >= DESBLOQUEADO) {
    gestionarEncendidoTactil();
  }
  
  // 3. LECTURA DE TEMPERATURA (siempre lee, pero solo calcula PWM) Y PRESIÓN (BMP280)
  if (now - lastTempMillis >= TEMP_INTERVAL) {
    lastTempMillis = now;
    temperatura_actual = leerTemperatura();
    presion_hpa = leerPresion();
    calcularPWMVentilador(temperatura_actual);
  }
  
  // 4. CONTROL DE FAROS (solo si ENCENDIDO)
  if (now - lastLuzMillis >= LUZ_INTERVAL) {
    lastLuzMillis = now;
    luz_actual = analogRead(LUZ_PIN);
    
    if (car_state == ENCENDIDO) {
      controlarFaros(luz_actual);
    } else {
      // Si no está encendido, faros apagados
      if (faros_encendidos) {
        apagarFaros();
      }
    }
  }
  
  // 5. ASISTENCIA DE RETROCESO (siempre mide, buzzer solo si ENCENDIDO)
  if (now - lastDistMillis >= DIST_INTERVAL) {
    lastDistMillis = now;
    asistenciaRetroceso();
  }
  
  // 6. CONTROL DEL MOTOR (según estado)
  actualizarMotor();

  // 7. ENVÍO A GOOGLE SHEETS CADA 10 SEGUNDOS

  if (now - lastSheetsMillis >= SHEETS_INTERVAL) {
    lastSheetsMillis = now;
    enviarDatosSheets();
}

}

// ===================================================================
// FUNCIÓN 1: GESTIÓN DE ACCESO CON RFID
// ===================================================================

void gestionarAccesoRFID() {
  // Verificar si hay una tarjeta presente
  if (!rfid.PICC_IsNewCardPresent()) {
    return;
  }
  
  // Verificar si se puede leer la tarjeta
  if (!rfid.PICC_ReadCardSerial()) {
    return;
  }
  
  // Comparar UID leído con UID válido
  if (compararUID(rfid.uid.uidByte, rfid.uid.size)) {
    // ACCESO CONCEDIDO
    if (car_state == BLOQUEADO) {
      car_state = DESBLOQUEADO;
      Serial.println("\n========================================");
      Serial.println("ACCESO CONCEDIDO - Carro DESBLOQUEADO");
      Serial.println("========================================");
      Serial.print("UID detectado: ");
      printUID(rfid.uid.uidByte, rfid.uid.size);
      
      // Encender LED Verde por 3 segundos
      // Importante: Detener SPI primero para liberar GPIO14
      rfid.PICC_HaltA();
      rfid.PCD_StopCrypto1();
      SPI.end();
      
      // Configurar GPIO14 como OUTPUT para LED
      pinMode(LED_VERDE, OUTPUT);
      digitalWrite(LED_VERDE, HIGH);
      Serial.println("LED Verde encendido por 3 segundos...");
      delay(3000);
      digitalWrite(LED_VERDE, LOW);
      Serial.println("LED Verde apagado.");
      
      // Reiniciar SPI para futuras lecturas RFID
      SPI.begin();
      rfid.PCD_Init();
      
      Serial.println("\nSistema desbloqueado. Presione sensor tactil para encender motor.\n");
      return; // Salir después de procesar
    }
  } else {
    // ACCESO DENEGADO
    Serial.println("\n========================================");
    Serial.println("ACCESO DENEGADO - UID no autorizado");
    Serial.println("========================================");
    Serial.print("UID detectado: ");
    printUID(rfid.uid.uidByte, rfid.uid.size);
    
    // Emitir alarma breve en buzzer
    digitalWrite(BUZZER_PIN, HIGH);
    delay(200);
    digitalWrite(BUZZER_PIN, LOW);
    delay(100);
    digitalWrite(BUZZER_PIN, HIGH);
    delay(200);
    digitalWrite(BUZZER_PIN, LOW);
    
    Serial.println("Sistema permanece BLOQUEADO.\n");
  }
  
  // Detener la lectura
  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
}

// ===================================================================
// FUNCIÓN 2: GESTIÓN DE ENCENDIDO CON SENSOR TÁCTIL
// ===================================================================

void gestionarEncendidoTactil() {
  bool touch_state = digitalRead(TOUCH_PIN);
  
  // Detección de flanco ascendente (toque nuevo)
  if (touch_state == HIGH && prev_touch_state == LOW) {
    delay(50); // Anti-rebote
    
    if (car_state == DESBLOQUEADO) {
      // Cambiar de DESBLOQUEADO a ENCENDIDO
      car_state = ENCENDIDO;
      Serial.println("\n========================================");
      Serial.println("MOTOR ENCENDIDO - Carro en marcha");
      Serial.println("========================================\n");
    } else if (car_state == ENCENDIDO) {
      // Cambiar de ENCENDIDO a DESBLOQUEADO
      car_state = DESBLOQUEADO;
      Serial.println("\n========================================");
      Serial.println("MOTOR APAGADO - Carro desbloqueado");
      Serial.println("========================================\n");
    }
  }
  
  prev_touch_state = touch_state;
}

// ===================================================================
// FUNCIÓN 3: CÁLCULO DE PWM VENTILADOR (solo variable interna)
// ===================================================================

void calcularPWMVentilador(float temp) {
  int pwm_anterior = pwm_ventilador_calculado;
  String nivel = "OFF";
  
  if (temp >= TEMP_MEDIO) {
    pwm_ventilador_calculado = PWM_ALTO;
    nivel = "ALTO (100%)";
  } else if (temp >= TEMP_BAJO) {
    pwm_ventilador_calculado = PWM_MEDIO;
    nivel = "MEDIO (60%)";
  } else if (temp >= TEMP_OFF) {
    pwm_ventilador_calculado = PWM_BAJO;
    nivel = "BAJO (20%)";
  } else {
    pwm_ventilador_calculado = PWM_OFF;
    nivel = "OFF";
  }
  
  // Solo imprimir si cambió el nivel
  if (pwm_anterior != pwm_ventilador_calculado) {
    Serial.print("Temperatura: ");
    Serial.print(temp, 1);
    Serial.print(" grados C | Nivel ventilador calculado: ");
    Serial.print(nivel);
    Serial.print(" (PWM: ");
    Serial.print(pwm_ventilador_calculado);
    Serial.println(") [Variable interna para Blynk]");
  }
}

// ===================================================================
// FUNCIÓN 4: CONTROL DE ILUMINACIÓN AUTOMÁTICA
// ===================================================================

void controlarFaros(int luz) {
  // Implementar histéresis para evitar parpadeo
  if (luz <= LUZ_OSCURO && !faros_encendidos) {
    // Está oscuro, encender faros
    digitalWrite(FARO_PIN, HIGH);
    faros_encendidos = true;
    Serial.print("Faros ENCENDIDOS | Luminosidad: ");
    Serial.println(luz);
  } else if (luz >= LUZ_CLARO && faros_encendidos) {
    // Está claro, apagar faros
    apagarFaros();
    Serial.print("Faros APAGADOS | Luminosidad: ");
    Serial.println(luz);
  }
}

void apagarFaros() {
  digitalWrite(FARO_PIN, LOW);
  faros_encendidos = false;
}

// ===================================================================
// FUNCIÓN 5: ASISTENCIA DE RETROCESO
// ===================================================================

void asistenciaRetroceso() {
  // Siempre mide distancia
  distancia_actual = medirDistancia();
  
  if (distancia_actual <= 0) {
    // Lectura inválida, apagar buzzer si estaba encendido
    if (car_state == ENCENDIDO) {
      digitalWrite(BUZZER_PIN, LOW);
    }
    return;
  }
  
  // Solo actuar con el buzzer si el carro está ENCENDIDO
  if (car_state == ENCENDIDO) {
    if (distancia_actual <= DIST_CRITICA) {
      // Distancia crítica: buzzer activo sonando constantemente
      digitalWrite(BUZZER_PIN, HIGH);
      Serial.print("ALERTA RETROCESO | Distancia: ");
      Serial.print(distancia_actual, 1);
      Serial.println(" cm | CRITICA - Buzzer constante");
    } else if (distancia_actual <= DIST_ALARMA) {
      // Distancia de advertencia: buzzer activo intermitente
      static unsigned long lastBeep = 0;
      unsigned long now = millis();
      if (now - lastBeep > 300) {
        digitalWrite(BUZZER_PIN, !digitalRead(BUZZER_PIN));
        lastBeep = now;
      }
      Serial.print("ALERTA RETROCESO | Distancia: ");
      Serial.print(distancia_actual, 1);
      Serial.println(" cm | Alarma intermitente");
    } else {
      // Distancia segura, apagar buzzer activo
      digitalWrite(BUZZER_PIN, LOW);
    }
  } else {
    // Si no está encendido, asegurar buzzer apagado
    digitalWrite(BUZZER_PIN, LOW);
  }
}

// ===================================================================
// FUNCIÓN 6: ACTUALIZAR ESTADO DEL MOTOR
// ===================================================================

void actualizarMotor() {
  if (car_state == ENCENDIDO) {
    // Motor ON
    digitalWrite(MOTOR_PIN, HIGH);
  } else {
    // Motor OFF (BLOQUEADO o DESBLOQUEADO)
    digitalWrite(MOTOR_PIN, LOW);
  }
}

// ===================================================================
// FUNCIÓN 7: ENVIAR DATOS A GOOGLE SHEETS
// ===================================================================
void enviarDatosSheets() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Sheets: WiFi NO conectado, no se envía.");
    return;
  }

  // Crear URL con los parámetros exactos de tu estructura
  String url = GOOGLE_SCRIPT_URL +
               "?presion=" + String(presion_hpa, 2) +
               "&temp_motor=" + String(temperatura_actual, 1) +
               "&luz=" + String(luz_actual) +
               "&dist=" + String(distancia_actual, 1) +
               "&estado=" + String(car_state);

  Serial.println("\nEnviando a Google Sheets:");
  Serial.println(url);

  WiFiClient client;

  if (!client.connect("script.google.com", 80)) {
    Serial.println("Error al conectar con Google Script");
    return;
  }

  // Enviar solicitud HTTP GET
  client.println("GET " + url + " HTTP/1.1");
  client.println("Host: script.google.com");
  client.println("Connection: close");
  client.println();

  // Leer respuesta
  while (client.connected() || client.available()) {
    if (client.available()) {
      String line = client.readStringUntil('\n');
      Serial.println(line);
    }
  }

  client.stop();
}


// ===================================================================
// FUNCIONES AUXILIARES
// ===================================================================

float leerTemperatura() {
  float temp = bmp.readTemperature();
  if (isnan(temp)) {
    Serial.println("Error al leer temperatura");
    return 0.0;
  }
  return temp;
}

float leerPresion() {
  float pres = bmp.readPressure() / 100.0F;  // Convertir a hPa
  if (isnan(pres)) {
    Serial.println("Error al leer presión");
    return 0.0;
  }
  return pres;
}


float medirDistancia() {
  // Enviar pulso de 10µs
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Leer tiempo de eco (timeout 30ms = ~5m)
  long duracion = pulseIn(ECHO_PIN, HIGH, 30000);
  
  if (duracion == 0) {
    return -1; // Timeout
  }
  
  // Calcular distancia: velocidad sonido = 343 m/s
  float distancia = duracion * 0.0343 / 2.0;
  return distancia;
}

bool compararUID(byte* uid, byte size) {
  if (size != 4) return false;
  
  for (byte i = 0; i < 4; i++) {
    if (uid[i] != UID_VALIDO[i]) {
      return false;
    }
  }
  return true;
}

void printUID(byte* uid, byte size) {
  for (byte i = 0; i < size; i++) {
    if (uid[i] < 0x10) Serial.print("0");
    Serial.print(uid[i], HEX);
    if (i < size - 1) Serial.print(":");
  }
  Serial.println();
}
