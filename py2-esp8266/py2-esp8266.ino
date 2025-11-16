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

// Switch de Retroceso
#define SWITCH_RETRO 12   // D6 - GPIO12

// Actuadores
#define MOTOR_PIN 1       // TX - GPIO1 (PWM)
#define BUZZER_PIN 10     // D9 (SD2) - GPIO10
#define FARO_IZQ 14       // D5 - GPIO14
#define FARO_DER 5        // D1 - GPIO5
#define LED_ROJO 15       // D8 - GPIO15
#define LED_VERDE 13      // D7 - GPIO13

// Sensor de Luz (Fotoresistencia)
#define LUZ_PIN A0        // A0 - ADC

// ===================================================================
// CONSTANTES DEL SISTEMA
// ===================================================================

// Estados del carro
#define BLOQUEADO 0
#define DESBLOQUEADO 1
#define ENCENDIDO 2

// Valores PWM del motor (0-1023)
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

// Estado del sistema
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
int luz_actual = 0;
float distancia_actual = 0.0;

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
  pinMode(SWITCH_RETRO, INPUT_PULLUP);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LUZ_PIN, INPUT);
  
  // Configurar pines de salida
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(FARO_IZQ, OUTPUT);
  pinMode(FARO_DER, OUTPUT);
  pinMode(LED_ROJO, OUTPUT);
  pinMode(LED_VERDE, OUTPUT);
  
  // Estado inicial: BLOQUEADO
  digitalWrite(LED_ROJO, HIGH);
  digitalWrite(LED_VERDE, LOW);
  digitalWrite(FARO_IZQ, LOW);
  digitalWrite(FARO_DER, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  analogWrite(MOTOR_PIN, PWM_OFF);
  
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
  
  Serial.println("\nSistema listo. Estado: BLOQUEADO\n");
}

// ===================================================================
// BUCLE PRINCIPAL
// ===================================================================

void loop() {
  unsigned long now = millis();
  
  // 1. GESTIÓN DE ACCESO Y ENCENDIDO
  gestionarAccesoRFID();
  gestionarEncendidoTactil();
  
  // 2. CONTROL TÉRMICO (solo si el carro está ENCENDIDO)
  if (now - lastTempMillis >= TEMP_INTERVAL) {
    lastTempMillis = now;
    temperatura_actual = leerTemperatura();
    if (car_state == ENCENDIDO) {
      controlarVentilador(temperatura_actual);
    } else {
      analogWrite(MOTOR_PIN, PWM_OFF);
    }
  }
  
  // 3. ILUMINACIÓN AUTOMÁTICA (si está DESBLOQUEADO o ENCENDIDO)
  if (now - lastLuzMillis >= LUZ_INTERVAL) {
    lastLuzMillis = now;
    luz_actual = analogRead(LUZ_PIN);
    if (car_state >= DESBLOQUEADO) {
      controlarFaros(luz_actual);
    } else {
      apagarFaros();
    }
  }
  
  // 4. ASISTENCIA DE RETROCESO
  if (now - lastDistMillis >= DIST_INTERVAL) {
    lastDistMillis = now;
    asistenciaRetroceso();
  }
  
  // Actualizar indicadores LED según estado
  actualizarIndicadoresEstado();
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
    if (car_state == BLOQUEADO) {
      car_state = DESBLOQUEADO;
      Serial.println("\nACCESO CONCEDIDO - Carro DESBLOQUEADO");
      Serial.print("UID detectado: ");
      printUID(rfid.uid.uidByte, rfid.uid.size);
    }
  } else {
    Serial.println("\nACCESO DENEGADO - UID no autorizado");
    Serial.print("UID detectado: ");
    printUID(rfid.uid.uidByte, rfid.uid.size);
    // Emitir alarma breve
    digitalWrite(BUZZER_PIN, HIGH);
    delay(200);
    digitalWrite(BUZZER_PIN, LOW);
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
      Serial.println("\nMotor ENCENDIDO");
    } else if (car_state == ENCENDIDO) {
      // Cambiar de ENCENDIDO a DESBLOQUEADO
      car_state = DESBLOQUEADO;
      Serial.println("\nMotor APAGADO (Carro aún desbloqueado)");
      analogWrite(MOTOR_PIN, PWM_OFF);
    }
    // Si está BLOQUEADO, no hace nada
  }
  
  prev_touch_state = touch_state;
}

// ===================================================================
// FUNCIÓN 3: CONTROL TÉRMICO DEL MOTOR VENTILADOR
// ===================================================================

void controlarVentilador(float temp) {
  int pwm_valor = PWM_OFF;
  String nivel = "OFF";
  
  if (temp >= TEMP_MEDIO) {
    pwm_valor = PWM_ALTO;
    nivel = "ALTO (100%)";
  } else if (temp >= TEMP_BAJO) {
    pwm_valor = PWM_MEDIO;
    nivel = "MEDIO (60%)";
  } else if (temp >= TEMP_OFF) {
    pwm_valor = PWM_BAJO;
    nivel = "BAJO (20%)";
  }
  
  analogWrite(MOTOR_PIN, pwm_valor);
  
  Serial.print("Temperatura: ");
  Serial.print(temp, 1);
  Serial.print(" grados C | Ventilador: ");
  Serial.print(nivel);
  Serial.print(" (PWM: ");
  Serial.print(pwm_valor);
  Serial.println(")");
}

// ===================================================================
// FUNCIÓN 4: CONTROL DE ILUMINACIÓN AUTOMÁTICA
// ===================================================================

void controlarFaros(int luz) {
  // Implementar histéresis para evitar parpadeo
  if (luz <= LUZ_OSCURO && !faros_encendidos) {
    // Está oscuro, encender faros
    digitalWrite(FARO_IZQ, HIGH);
    digitalWrite(FARO_DER, HIGH);
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
  digitalWrite(FARO_IZQ, LOW);
  digitalWrite(FARO_DER, LOW);
  faros_encendidos = false;
}

// ===================================================================
// FUNCIÓN 5: ASISTENCIA DE RETROCESO
// ===================================================================

void asistenciaRetroceso() {
  // Leer estado del switch de retroceso
  bool modo_retroceso = (digitalRead(SWITCH_RETRO) == LOW); // Pull-up activo en LOW
  
  if (!modo_retroceso) {
    // Si no está en retroceso, apagar buzzer
    digitalWrite(BUZZER_PIN, LOW);
    return;
  }
  
  // Medir distancia con HC-SR04
  distancia_actual = medirDistancia();
  
  if (distancia_actual <= 0) {
    // Lectura inválida
    digitalWrite(BUZZER_PIN, LOW);
    return;
  }
  
  Serial.print("Retroceso activo | Distancia: ");
  Serial.print(distancia_actual, 1);
  Serial.print(" cm");
  
  if (distancia_actual <= DIST_CRITICA) {
    // Distancia crítica: alarma constante
    digitalWrite(BUZZER_PIN, HIGH);
    Serial.println(" | ALARMA CRÍTICA");
  } else if (distancia_actual <= DIST_ALARMA) {
    // Distancia de advertencia: alarma intermitente
    static unsigned long lastBeep = 0;
    unsigned long now = millis();
    if (now - lastBeep > 500) {
      digitalWrite(BUZZER_PIN, !digitalRead(BUZZER_PIN));
      lastBeep = now;
    }
    Serial.println(" | Alarma intermitente");
  } else {
    // Distancia segura
    digitalWrite(BUZZER_PIN, LOW);
    Serial.println(" | Distancia segura");
  }
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

void actualizarIndicadoresEstado() {
  switch (car_state) {
    case BLOQUEADO:
      digitalWrite(LED_ROJO, HIGH);
      digitalWrite(LED_VERDE, LOW);
      break;
    case DESBLOQUEADO:
      digitalWrite(LED_ROJO, LOW);
      digitalWrite(LED_VERDE, HIGH);
      break;
    case ENCENDIDO:
      digitalWrite(LED_ROJO, LOW);
      digitalWrite(LED_VERDE, HIGH); // Verde parpadea o permanece encendido
      break;
  }
}

