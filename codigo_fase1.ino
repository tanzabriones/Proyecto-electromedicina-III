#include <Wire.h>
#include <SPI.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <MPU6050_light.h>
#include <MAX30105.h>
#include <MFRC522.h>

// ============================================================================
// DEFINICIÓN DE PINES
// ============================================================================
#define PIN_DS18B20   15   // Bus OneWire
#define PIN_SDA       21   // Bus I2C Compartido (MPU6050 + MAX30102)
#define PIN_SCL       22   // Bus I2C Compartido (MPU6050 + MAX30102)

#define PIN_RFID_SS   5    // SDA / SS del RC522 (Bus SPI)
#define PIN_RFID_RST  4    // RST del RC522

#define PIN_LED_OK    2    // LED Verde (GPIO 2)
#define PIN_LED_DENY  13   // LED Rojo 
#define UMBRAL_TEMP_ALTA 25.0 // Umbral de alerta térmica en °C

// Instancias de Hardware
OneWire oneWire(PIN_DS18B20);
DallasTemperature sensorTemp(&oneWire);
MPU6050 mpu(Wire);
MAX30105 oximetro;
MFRC522 rfid(PIN_RFID_SS, PIN_RFID_RST);

// Semáforos FreeRTOS
SemaphoreHandle_t xI2CMutex;
SemaphoreHandle_t xDataMutex;
SemaphoreHandle_t xSPIMutex;

// Banderas de inicialización
bool okDS18B20  = false;
bool okMPU6050  = false;
bool okMAX30102 = false;
bool okRFID     = false;

volatile bool sesionActiva = false;
String paramedicoActual = "";

// Lista blanca autorizada (UIDs de tus tarjetas)
const int NUM_TAGS = 2;
const String TAGS_AUTORIZADOS[NUM_TAGS] = {
  "13 C1 45 13",
  "07 7B 0A 73"
};

// Estructura de almacenamiento de telemetría unificada
struct TelemetriaGlobal {
  float ax, ay, az;
  float roll, pitch, yaw;
  uint32_t irVal, redVal;
  bool dedoContacto;
  float temperatura;
  bool tempValida;
} datosSistema;

// ============================================================================
// TAREA 1: ACCESO RFID Y ACTUADORES LOCALES (Core 1)
// ============================================================================
void vTareaRFID(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriodo = pdMS_TO_TICKS(100);

  for (;;) {
    if (okRFID) {
      bool hayTarjeta = false;
      bool lecturaOk = false;

      // Lectura segura en el bus SPI
      if (xSemaphoreTake(xSPIMutex, pdMS_TO_TICKS(25)) == pdTRUE) {
        if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
          hayTarjeta = true;
          lecturaOk = true;
        }
        xSemaphoreGive(xSPIMutex);
      }

      if (hayTarjeta && lecturaOk) {
        String uidStr = "";
        for (byte i = 0; i < rfid.uid.size; i++) {
          if (rfid.uid.uidByte[i] < 0x10) uidStr += "0";
          uidStr += String(rfid.uid.uidByte[i], HEX);
          if (i < rfid.uid.size - 1) uidStr += " ";
        }
        uidStr.toUpperCase();

        bool autorizado = false;
        for (int i = 0; i < NUM_TAGS; i++) {
          if (uidStr == TAGS_AUTORIZADOS[i]) {
            autorizado = true;
            break;
          }
        }

        if (autorizado) {
          digitalWrite(PIN_LED_OK, HIGH);
          digitalWrite(PIN_LED_DENY, LOW);

          if (!sesionActiva) {
            sesionActiva = true;
            paramedicoActual = uidStr;
            Serial.println("\n*****************************************************************");
            Serial.printf(" [RFID AUTORIZADO] SESIÓN INICIADA -> Operador UID: %s\n", uidStr.c_str());
            Serial.println("*****************************************************************");
          } else if (paramedicoActual == uidStr) {
            sesionActiva = false;
            paramedicoActual = "";
            Serial.println("\n*****************************************************************");
            Serial.printf(" [RFID AUTORIZADO] SESIÓN CERRADA  -> Operador UID: %s\n", uidStr.c_str());
            Serial.println("*****************************************************************");
          }

          vTaskDelay(pdMS_TO_TICKS(400));
          digitalWrite(PIN_LED_OK, LOW);
        } else {
          digitalWrite(PIN_LED_DENY, HIGH);
          digitalWrite(PIN_LED_OK, LOW);

          Serial.println("\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
          Serial.printf(" [ALERTA DE SEGURIDAD] ACCESO DENEGADO -> UID: %s\n", uidStr.c_str());
          Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

          vTaskDelay(pdMS_TO_TICKS(400));
          digitalWrite(PIN_LED_DENY, LOW);
        }

        if (xSemaphoreTake(xSPIMutex, pdMS_TO_TICKS(25)) == pdTRUE) {
          rfid.PICC_HaltA();
          rfid.PCD_StopCrypto1();
          xSemaphoreGive(xSPIMutex);
        }
      }
    }

    vTaskDelayUntil(&xLastWakeTime, xPeriodo);
  }
}

// ============================================================================
// TAREA 2: CINEMÁTICA MPU6050 (Core 1 - 20 Hz continuo)
// ============================================================================
void vTareaCinematica(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriodo = pdMS_TO_TICKS(50); // Muestreo cada 50 ms

  for (;;) {
    if (sesionActiva && okMPU6050) {
      if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(15)) == pdTRUE) {
        mpu.update();
        float _ax = mpu.getAccX(), _ay = mpu.getAccY(), _az = mpu.getAccZ();
        float _roll = mpu.getAngleX(), _pitch = mpu.getAngleY(), _yaw = mpu.getAngleZ();
        xSemaphoreGive(xI2CMutex);

        if (xSemaphoreTake(xDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          datosSistema.ax = _ax; datosSistema.ay = _ay; datosSistema.az = _az;
          datosSistema.roll = _roll; datosSistema.pitch = _pitch; datosSistema.yaw = _yaw;
          xSemaphoreGive(xDataMutex);
        }
      }
    }
    vTaskDelayUntil(&xLastWakeTime, xPeriodo);
  }
}

// ============================================================================
// TAREA 3: PULSIOXIMETRÍA MAX30102 (Core 1 - 25 Hz continuo)
// ============================================================================
void vTareaPulsioximetro(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriodo = pdMS_TO_TICKS(40); // Muestreo cada 40 ms

  for (;;) {
    if (sesionActiva && okMAX30102) {
      uint32_t _ir = 0, _red = 0;
      bool nuevoDato = false;

      if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(15)) == pdTRUE) {
        oximetro.check();
        if (oximetro.available()) {
          _ir = oximetro.getFIFOIR();
          _red = oximetro.getFIFORed();
          oximetro.nextSample();
          nuevoDato = true;
        }
        xSemaphoreGive(xI2CMutex);
      }

      if (nuevoDato) {
        if (xSemaphoreTake(xDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          datosSistema.irVal = _ir;
          datosSistema.redVal = _red;
          datosSistema.dedoContacto = (_ir >= 40000);
          xSemaphoreGive(xDataMutex);
        }
      }
    }
    vTaskDelayUntil(&xLastWakeTime, xPeriodo);
  }
}

// ============================================================================
// TAREA 4: TEMPERATURA CLÍNICA DS18B20 (Core 1 - 1 Hz)
// ============================================================================
// ============================================================================
// TAREA 4: TEMPERATURA CLÍNICA DS18B20 (Core 1 - 1 Hz)
// ============================================================================
void vTareaTemperatura(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriodo = pdMS_TO_TICKS(1000); // 1 segundo

  for (;;) {
    if (sesionActiva && okDS18B20) {
      float t = sensorTemp.getTempCByIndex(0);
      sensorTemp.requestTemperatures(); // Petición asíncrona para la siguiente muestra

      bool val = (t != DEVICE_DISCONNECTED_C && t >= 10.0 && t <= 45.0);

      // --- CONDICIÓN DE ALERTA POR TEMPERATURA > 25 °C ---
      if (val && t > UMBRAL_TEMP_ALTA) {
        digitalWrite(PIN_LED_DENY, HIGH); // Enciende LED rojo de alerta
      } else {
        // Solo apaga el LED rojo si la sesión sigue activa y no hay un rechazo RFID en curso
        digitalWrite(PIN_LED_DENY, LOW);  // Apaga LED rojo si la temp es normal (<= 25 °C)
      }

      if (xSemaphoreTake(xDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        datosSistema.temperatura = t;
        datosSistema.tempValida = val;
        xSemaphoreGive(xDataMutex);
      }
    } else {
      // Si la sesión no está activa, asegurar que el LED de alerta permanezca apagado
      if (!sesionActiva) {
        digitalWrite(PIN_LED_DENY, LOW);
      }
    }
    vTaskDelayUntil(&xLastWakeTime, xPeriodo);
  }
}

// ============================================================================
// TAREA 5: CONSOLA TRANQUILA Y PAUSADA (Cada 1.5 segundos)
// ============================================================================
void vTareaConsolaPausada(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriodo = pdMS_TO_TICKS(1500); // Actualización visual cada 1.5 s

  TelemetriaGlobal d;

  for (;;) {
    unsigned long ahora = millis();

    if (!sesionActiva) {
      Serial.println("\n-----------------------------------------------------------------");
      Serial.printf(" [MODO ESPERA] Tiempo: %lu ms | ESTADO: SESIÓN BLOQUEADA\n", ahora);
      Serial.println("  -> Acerque su tarjeta RFID autorizada para activar la adquisición.");
      Serial.println("-----------------------------------------------------------------");
    } else {
      if (xSemaphoreTake(xDataMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        d = datosSistema;
        xSemaphoreGive(xDataMutex);
      }

      Serial.println("\n=================================================================");
      Serial.printf("  MONITOR CLÍNICO DE AMBULANCIA (FASE 1) | Operador: %s\n", paramedicoActual.c_str());
      Serial.printf("  Marca Temporal: %lu ms\n", ahora);
      Serial.println("=================================================================");
      
      // 1. Monitor de Temperatura con alerta
      Serial.print(" [1] TEMPERATURA (DS18B20) : ");
      if (d.temperatura == DEVICE_DISCONNECTED_C) {
        Serial.println("DESCONECTADO / ERROR BUS");
      } else {
        if (d.temperatura > UMBRAL_TEMP_ALTA) {
          Serial.printf("%.2f °C | ALERTA: TEMPERATURA ELEVADA (> 25°C) [LED ROJO ON]\n", d.temperatura);
        } else {
          Serial.printf("%.2f °C | Estado: %s\n", d.temperatura, d.tempValida ? "NORMAL (VÁLIDO)" : "FUERA DE RANGO");
        }
      }

      // 2. Monitor Fotopletismográfico
      Serial.print(" [2] PULSO / PPG (MAX30102) : ");
      if (d.dedoContacto) {
        Serial.printf("IR: %u | RED: %u | Contacto: DEDO DETECTADO [OK]\n", d.irVal, d.redVal);
      } else {
        Serial.printf("IR: %u | RED: %u | Contacto: SIN CONTACTO (Coloque el dedo)\n", d.irVal, d.redVal);
      }

      // 3. Monitor Cinemático
      Serial.println(" [3] CINEMÁTICA  (MPU6050) :");
      Serial.printf("     -> Aceleración (g) : X: %+.2f | Y: %+.2f | Z: %+.2f\n", d.ax, d.ay, d.az);
      Serial.printf("     -> Inclinación     : Roll: %+.1f° | Pitch: %+.1f° | Yaw: %+.1f°\n", d.roll, d.pitch, d.yaw);
      Serial.println("=================================================================");
    }

    vTaskDelayUntil(&xLastWakeTime, xPeriodo);
  }
}

// ============================================================================
// SETUP: CONFIGURACIÓN E INICIALIZACIÓN DE HARDWARE
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(1200);

  pinMode(PIN_LED_OK, OUTPUT);
  pinMode(PIN_LED_DENY, OUTPUT);
  digitalWrite(PIN_LED_OK, LOW);
  digitalWrite(PIN_LED_DENY, LOW);

  // Semáforos de sincronización
  xI2CMutex  = xSemaphoreCreateMutex();
  xDataMutex = xSemaphoreCreateMutex();
  xSPIMutex  = xSemaphoreCreateMutex();

  // Limpieza inicial de variables
  datosSistema.ax = 0; datosSistema.ay = 0; datosSistema.az = 0;
  datosSistema.roll = 0; datosSistema.pitch = 0; datosSistema.yaw = 0;
  datosSistema.irVal = 0; datosSistema.redVal = 0;
  datosSistema.dedoContacto = false;
  datosSistema.temperatura = 0;
  datosSistema.tempValida = false;

  Serial.println("\n--- INICIALIZANDO DISPOSITIVOS FASE 1 ---");

  // 1. Bus I2C Compartido (MPU6050 + MAX30102)
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);

  // 2. MPU6050
  if (mpu.begin() == 0) {
    okMPU6050 = true;
    mpu.calcOffsets();
    Serial.println("[OK] MPU6050 calibrado.");
  } else {
    Serial.println("[FALLO] MPU6050 no responde en bus I2C.");
  }

  // 3. MAX30102
  if (oximetro.begin(Wire, I2C_SPEED_STANDARD)) {
    oximetro.setup(60, 4, 2, 100, 411, 4096);
    okMAX30102 = true;
    Serial.println("[OK] MAX30102 configurado.");
  } else {
    Serial.println("[FALLO] MAX30102 no responde en bus I2C.");
  }

  // 4. DS18B20 (Bus OneWire)
  pinMode(PIN_DS18B20, INPUT_PULLUP);
  sensorTemp.begin();
  if (sensorTemp.getDeviceCount() > 0) {
    okDS18B20 = true;
    sensorTemp.setWaitForConversion(false);
    sensorTemp.requestTemperatures();
    Serial.println("[OK] DS18B20 detectado en GPIO 15.");
  } else {
    Serial.println("[FALLO] DS18B20 no encontrado en GPIO 15.");
  }

  // 5. Lector RFID-RC522 (Bus SPI)
  SPI.begin(); // SCK=18, MISO=19, MOSI=23 por defecto
  rfid.PCD_Init();
  delay(50);
  rfid.PCD_SetAntennaGain(rfid.RxGain_max);
  rfid.PCD_AntennaOn();

  byte v = rfid.PCD_ReadRegister(rfid.VersionReg);
  if (v != 0x00 && v != 0xFF) {
    okRFID = true;
    Serial.printf("[OK] RFID RC522 detectado (VersionReg: 0x%02X) en D5.\n", v);
  } else {
    Serial.println("[FALLO] RC522 no responde en bus SPI (Revisar D5, D18, D19, D23 y 3V3).");
  }

  Serial.println("--- INICIALIZACIÓN COMPLETADA ---\n");

  // Creación de tareas independientes en Core 1
  xTaskCreatePinnedToCore(vTareaRFID,          "TareaRFID",      3584, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(vTareaCinematica,    "TareaIMU",       3072, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(vTareaPulsioximetro, "TareaPulsiox",   3072, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(vTareaTemperatura,   "TareaTemp",      2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(vTareaConsolaPausada, "TareaConsola",   3072, NULL, 1, NULL, 1);
}

void loop() {
  vTaskDelete(NULL);
}
