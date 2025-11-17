
// ===================================================================
// AXIS MOTORS - Sistema de Carro Inteligente IoT
// Grupo 6: Diego C. S., Pedro C. T., Hugo M. L., Arodi C. R.
// Programacion de Microprocesadores - UVG
// ===================================================================

#include <Wire.h>
// #include <Adafruit_Sensor.h>
// #include <Adafruit_BMP280.h>

// ===================================================================
// DEFINICION DE PINES
// ===================================================================

// BMP280
// #define BMP_SDA 4         // D2 - GPIO4
// #define BMP_SCL 5         // D1 - GPIO5

// Microfono KY-037/038 (Analogico)
#define MIC_ANALOG A0     // A0 - ADC (nivel de ruido)

// Fotorresistencia (Digital)
#define LUZ_DO 16         // D0 - GPIO16 (deteccion luz/oscuro)

// Sensor Ultrasonico HC-SR04
#define TRIG_PIN 14       // D5 - GPIO14
#define ECHO_PIN 13       // D7 - GPIO13

// Sensor Tactil TTP223
#define TOUCH_PIN 12      // D6 - GPIO12

// Actuadores (con resistencias para boot seguro)
#define MOTOR_PIN 0       // D3 - GPIO0 (requiere pull-up 10k a 3.3V)
#define BUZZER_PIN 15     // D8 - GPIO15 (requiere pull-down 10k a GND)
#define FARO_PIN 2        // D4 - GPIO2 (requiere pull-up 10k a 3.3V)

// ===================================================================
// CONSTANTES DEL SISTEMA
// ===================================================================

// Estados del carro
#define APAGADO 0       
#define ENCENDIDO 1       

// BMP280 
// #define TEMP_OFF 30.0
// #define TEMP_BAJO 35.0
// #define TEMP_MEDIO 40.0

// Umbrales de distancia
#define DIST_ALARMA 30
#define DIST_CRITICA 10

// Tiempos de actualizacion (ms)
// #define TEMP_INTERVAL 2000
#define LUZ_INTERVAL 500
#define DIST_INTERVAL 200
#define MIC_INTERVAL 300

// ===================================================================
// VARIABLES GLOBALES
// ===================================================================

int car_state = APAGADO;
// Adafruit_BMP280 bmp;  // BMP280 deshabilitado

// unsigned long lastTempMillis = 0;
unsigned long lastLuzMillis = 0;
unsigned long lastDistMillis = 0;
unsigned long lastMicMillis = 0;

bool prev_touch_state = false;
bool faros_encendidos = false;

// float temperatura_actual = 0.0;  // BMP280 deshabilitado
bool luz_oscura = false;
float distancia_actual = 0.0;
int nivel_ruido = 0;

// ===================================================================
// SETUP
// ===================================================================

void setup() {
  Serial.begin(115200);
  delay(100);
  
  Serial.println("\n===================================");
  Serial.println("AXIS MOTORS - Sistema Iniciando...");
  Serial.println("===================================\n");
  
  // Configurar pines de entrada
  pinMode(TOUCH_PIN, INPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LUZ_DO, INPUT);
  
  // Configurar pines de salida
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(FARO_PIN, OUTPUT);
  
  // Estado inicial: todo apagado
  digitalWrite(MOTOR_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(FARO_PIN, LOW);
  
  // BMP280 deshabilitado - sensor danado
  /*
  // Inicializar I2C para BMP280
  Wire.begin(BMP_SDA, BMP_SCL);
  delay(100);
  
  Serial.println("Intentando inicializar BMP280...");
  
  if (!bmp.begin(0x76)) {
    Serial.println("BMP280 no detectado en direccion 0x76");
    Serial.println("Probando direccion 0x77...");
    if (!bmp.begin(0x77)) {
      Serial.println("ERROR: BMP280 no detectado en ninguna direccion");
      Serial.println("Verifica las conexiones:");
      Serial.println("  SDA -> D2 (GPIO4)");
      Serial.println("  SCL -> D1 (GPIO5)");
      Serial.println("  VCC -> 3.3V");
      Serial.println("  GND -> GND");
    } else {
      Serial.println("BMP280 inicializado correctamente en 0x77");
      bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                      Adafruit_BMP280::SAMPLING_X2,
                      Adafruit_BMP280::SAMPLING_X16,
                      Adafruit_BMP280::FILTER_X16,
                      Adafruit_BMP280::STANDBY_MS_500);
    }
  } else {
    Serial.println("BMP280 inicializado correctamente en 0x76");
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);
  }
  */
  
  Serial.println("NOTA: BMP280 deshabilitado (sensor danado)");
  
  Serial.println("\nConfiguracion:");
  Serial.println("- Microfono: A0 (Analogico - nivel de ruido)");
  Serial.println("- Fotorresistencia: D0 (Digital - deteccion luz)");
  Serial.println("\nSistema listo.");
  Serial.println("ESTADO: APAGADO");
  Serial.println("Presiona sensor tactil para encender\n");
}

// ===================================================================
// LOOP
// ===================================================================

void loop() {
  unsigned long now = millis();
  
  // 1. Sensor Tactil (control de encendido/apagado)
  gestionarEncendidoTactil();
  
  // 2. Temperatura - DESHABILITADO
  /*
  if (now - lastTempMillis >= TEMP_INTERVAL) {
    lastTempMillis = now;
    leerTemperatura();
  }
  */
  
  // 3. Fotorresistencia y Faros
  if (now - lastLuzMillis >= LUZ_INTERVAL) {
    lastLuzMillis = now;
    leerFotorresistencia();
    
    if (car_state == ENCENDIDO) {
      controlarFaros();
    } else {
      if (faros_encendidos) apagarFaros();
    }
  }
  
  // 4. Distancia y Asistencia de Retroceso
  if (now - lastDistMillis >= DIST_INTERVAL) {
    lastDistMillis = now;
    asistenciaRetroceso();
  }
  
  // 5. Microfono (nivel de ruido analogico)
  if (now - lastMicMillis >= MIC_INTERVAL) {
    lastMicMillis = now;
    if (car_state == ENCENDIDO) {
      leerNivelRuido();
    }
  }
  
  // 6. Motor
  actualizarMotor();
}

// ===================================================================
// FUNCIONES
// ===================================================================

void gestionarEncendidoTactil() {
  bool touch_state = digitalRead(TOUCH_PIN);
  
  if (touch_state == HIGH && prev_touch_state == LOW) {
    delay(50); // Debounce
    
    if (car_state == APAGADO) {
      car_state = ENCENDIDO;
      Serial.println("\n========================================");
      Serial.println("MOTOR ENCENDIDO");
      Serial.println("========================================\n");
    } else if (car_state == ENCENDIDO) {
      car_state = APAGADO;
      Serial.println("\n========================================");
      Serial.println("MOTOR APAGADO");
      Serial.println("========================================\n");
    }
  }
  
  prev_touch_state = touch_state;
}

// BMP280 deshabilitado
/*
void leerTemperatura() {
  float temp = bmp.readTemperature();
  
  // Verificar si la lectura es valida
  if (isnan(temp) || temp < -40 || temp > 85) {
    Serial.println("Error leyendo temperatura - Verificar conexiones BMP280");
    Serial.println("Revisa: SDA en D2 (GPIO4), SCL en D1 (GPIO5), VCC en 3.3V, GND");
    return;
  }
  
  temperatura_actual = temp;
  
  Serial.print("Temperatura: ");
  Serial.print(temp, 1);
  Serial.println(" C");
}
*/

void leerFotorresistencia() {
  // Leer pin digital de fotorresistencia
  // HIGH = oscuro (activar faros)
  // LOW = claro (apagar faros)
  bool estado_luz = digitalRead(LUZ_DO);
  
  if (estado_luz == HIGH) {
    // Oscuro
    if (!luz_oscura) {
      luz_oscura = true;
      Serial.println("Luz: OSCURO detectado");
    }
  } else {
    // Claro
    if (luz_oscura) {
      luz_oscura = false;
      Serial.println("Luz: CLARO detectado");
    }
  }
}

void controlarFaros() {
  if (luz_oscura && !faros_encendidos) {
    digitalWrite(FARO_PIN, HIGH);
    faros_encendidos = true;
    Serial.println("Faros ENCENDIDOS");
  } else if (!luz_oscura && faros_encendidos) {
    apagarFaros();
    Serial.println("Faros APAGADOS");
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
    if (distancia_actual <= DIST_CRITICA) {
      digitalWrite(BUZZER_PIN, HIGH);
      Serial.print("ALERTA CRITICA | Distancia: ");
      Serial.print(distancia_actual, 1);
      Serial.println(" cm");
    } else if (distancia_actual <= DIST_ALARMA) {
      // Beep intermitente
      static unsigned long lastBeep = 0;
      unsigned long now = millis();
      if (now - lastBeep > 300) {
        digitalWrite(BUZZER_PIN, !digitalRead(BUZZER_PIN));
        lastBeep = now;
      }
    } else {
      digitalWrite(BUZZER_PIN, LOW);
    }
  } else {
    digitalWrite(BUZZER_PIN, LOW);
  }
}

void leerNivelRuido() {
  // Leer valor analogico del microfono con promedio para filtrar ruido electrico
  int suma = 0;
  int lecturas = 10;
  
  for (int i = 0; i < lecturas; i++) {
    suma += analogRead(MIC_ANALOG);
    delayMicroseconds(100);
  }
  
  nivel_ruido = suma / lecturas;
  
  Serial.print("Nivel de ruido: ");
  Serial.print(nivel_ruido);
  Serial.print(" | ");
  
  // Barra visual del nivel
  int barras = map(nivel_ruido, 0, 1023, 0, 50);
  for (int i = 0; i < barras; i++) {
    Serial.print("=");
  }
  Serial.print(" ");
  
  // Clasificacion del ruido
  if (nivel_ruido < 100) {
    Serial.println("Silencio");
  } else if (nivel_ruido < 300) {
    Serial.println("Ruido bajo");
  } else if (nivel_ruido < 500) {
    Serial.println("Ruido moderado");
  } else if (nivel_ruido < 700) {
    Serial.println("Ruido alto");
  } else {
    Serial.println("Ruido muy alto - PELIGROSO");
  }
}

void actualizarMotor() {
  if (car_state == ENCENDIDO) {
    digitalWrite(MOTOR_PIN, HIGH);
  } else {
    digitalWrite(MOTOR_PIN, LOW);
  }
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
