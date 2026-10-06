#include <Wire.h>
#include <SPI.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <MPU6050_light.h>
#include <MAX30105.h>
#include <MFRC522.h>

// ==========================================
// DEFINICIÓN DE PINES
// ==========================================
#define PIN_DS18B20   15
#define PIN_SDA       21
#define PIN_SCL       22
#define PIN_RFID_SS   5
#define PIN_RFID_RST  4

// Actuadores locales exigidos por la rúbrica
#define PIN_LED_OK    2    // LED verde / indicador de acceso concedido
#define PIN_LED_DENY  13   // LED rojo / indicador de acceso denegado

// Periféricos
OneWire oneWire(PIN_DS18B20);
DallasTemperature sensorTemp(&oneWire);
MPU6050 imu(Wire);
MAX30105 oximetro;
MFRC522 rfid(PIN_RFID_SS, PIN_RFID_RST);

// Semáforos y sincronización FreeRTOS
SemaphoreHandle_t xI2CMutex;
SemaphoreHandle_t xSerialMutex;

// Variables de estado
bool okDS18B20  = false;
bool okMPU6050  = false;
bool okMAX30102 = false;
bool okRFID     = false;

volatile bool sesionActiva = false;
String paramedicoActual = "";

// Lista Blanca Autorizada
const int NUM_TAGS = 2;
const String TAGS_AUTORIZADOS[NUM_TAGS] = {
  "13 C1 45 13",
  "0C F6 17 29"
};

// ==========================================
// TAREA 1: ACCESO RFID Y ACTUADORES LOCALES
// ==========================================
void vTareaRFID(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriodo = pdMS_TO_TICKS(100);

  for (;;) {
    if (okRFID && rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
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

      unsigned long ahora = millis();

      if (autorizado) {
        digitalWrite(PIN_LED_OK, HIGH);
        digitalWrite(PIN_LED_DENY, LOW);

        if (!sesionActiva) {
          sesionActiva = true;
          paramedicoActual = uidStr;
          if (xSemaphoreTake(xSerialMutex, portMAX_DELAY) == pdTRUE) {
            Serial.printf("{\"sensor\":\"RC522\",\"auth\":true,\"msg\":\"SESION INICIADA\",\"uid\":\"%s\",\"ts\":%lu}\n", uidStr.c_str(), ahora);
            xSemaphoreGive(xSerialMutex);
          }
        } else if (paramedicoActual == uidStr) {
          sesionActiva = false;
          paramedicoActual = "";
          if (xSemaphoreTake(xSerialMutex, portMAX_DELAY) == pdTRUE) {
            Serial.printf("{\"sensor\":\"RC522\",\"auth\":true,\"msg\":\"SESION CERRADA\",\"uid\":\"%s\",\"ts\":%lu}\n", uidStr.c_str(), ahora);
            xSemaphoreGive(xSerialMutex);
          }
        }
        vTaskDelay(pdMS_TO_TICKS(400));
        digitalWrite(PIN_LED_OK, LOW);
      } else {
        digitalWrite(PIN_LED_DENY, HIGH);
        digitalWrite(PIN_LED_OK, LOW);

        if (xSemaphoreTake(xSerialMutex, portMAX_DELAY) == pdTRUE) {
          Serial.printf("{\"sensor\":\"RC522\",\"auth\":false,\"msg\":\"ACCESO DENEGADO\",\"uid\":\"%s\",\"ts\":%lu}\n", uidStr.c_str(), ahora);
          xSemaphoreGive(xSerialMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(400));
        digitalWrite(PIN_LED_DENY, LOW);
      }

      rfid.PICC_HaltA();
      rfid.PCD_StopCrypto1();
    }

    vTaskDelayUntil(&xLastWakeTime, xPeriodo);
  }
}

// ==========================================
// TAREA 2: CINEMÁTICA MPU6050 (10 Hz)
// ==========================================
void vTareaCinematica(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriodo = pdMS_TO_TICKS(100);

  for (;;) {
    if (sesionActiva && okMPU6050) {
      if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        imu.update();
        float ax = imu.getAccX(), ay = imu.getAccY(), az = imu.getAccZ();
        float gx = imu.getGyroX(), gy = imu.getGyroY(), gz = imu.getGyroZ();
        xSemaphoreGive(xI2CMutex);

        unsigned long ahora = millis();
        if (xSemaphoreTake(xSerialMutex, portMAX_DELAY) == pdTRUE) {
          Serial.printf("{\"sensor\":\"MPU6050\",\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f,\"ts\":%lu}\n",
                        ax, ay, az, gx, gy, gz, ahora);
          xSemaphoreGive(xSerialMutex);
        }
      }
    }
    vTaskDelayUntil(&xLastWakeTime, xPeriodo);
  }
}

// ==========================================
// TAREA 3: PULSIOXIMETRÍA MAX30102 (25 Hz)
// ==========================================
void vTareaPulsioximetro(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriodo = pdMS_TO_TICKS(40);

  for (;;) {
    if (sesionActiva && okMAX30102) {
      uint32_t irVal = 0, redVal = 0;
      bool hayMuestra = false;

      if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(15)) == pdTRUE) {
        oximetro.check();
        if (oximetro.available()) {
          irVal = oximetro.getFIFOIR();
          redVal = oximetro.getFIFORed();
          oximetro.nextSample();
          hayMuestra = true;
        }
        xSemaphoreGive(xI2CMutex);
      }

      if (hayMuestra) {
        unsigned long ahora = millis();
        bool contacto = (irVal > 15000);
        if (xSemaphoreTake(xSerialMutex, portMAX_DELAY) == pdTRUE) {
          Serial.printf("{\"sensor\":\"MAX30102\",\"ir\":%u,\"red\":%u,\"contact\":%s,\"ts\":%lu}\n",
                        irVal, redVal, contacto ? "true" : "false", ahora);
          xSemaphoreGive(xSerialMutex);
        }
      }
    }
    vTaskDelayUntil(&xLastWakeTime, xPeriodo);
  }
}

// ==========================================
// TAREA 4: TEMPERATURA DS18B20 (1 Hz)
// ==========================================
void vTareaTemperatura(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriodo = pdMS_TO_TICKS(1000);

  for (;;) {
    if (sesionActiva && okDS18B20) {
      float t = sensorTemp.getTempCByIndex(0);
      sensorTemp.requestTemperatures(); // Petición no bloqueante previa

      unsigned long ahora = millis();
      bool valida = (t != DEVICE_DISCONNECTED_C && t >= 15.0 && t <= 45.0);

      if (xSemaphoreTake(xSerialMutex, portMAX_DELAY) == pdTRUE) {
        Serial.printf("{\"sensor\":\"DS18B20\",\"temp\":%.2f,\"valid\":%s,\"ts\":%lu}\n",
                      t, valida ? "true" : "false", ahora);
        xSemaphoreGive(xSerialMutex);
      }
    }
    vTaskDelayUntil(&xLastWakeTime, xPeriodo);
  }
}

// ==========================================
// SETUP PRINCIPAL
// ==========================================
void setup() {
  Serial.begin(115200);
  
  pinMode(PIN_LED_OK, OUTPUT);
  pinMode(PIN_LED_DENY, OUTPUT);
  digitalWrite(PIN_LED_OK, LOW);
  digitalWrite(PIN_LED_DENY, LOW);

  // Semáforos para evitar colisiones
  xI2CMutex = xSemaphoreCreateMutex();
  xSerialMutex = xSemaphoreCreateMutex();

  // 1. Bus I2C Compartido
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);

  // 2. DS18B20
  pinMode(PIN_DS18B20, INPUT_PULLUP);
  sensorTemp.begin();
  if (sensorTemp.getDeviceCount() > 0) {
    okDS18B20 = true;
    sensorTemp.setWaitForConversion(false);
    sensorTemp.requestTemperatures();
  }

  // 3. MPU6050
  if (imu.begin() == 0) {
    okMPU6050 = true;
    imu.calcOffsets();
  }

  // 4. MAX30102
  if (oximetro.begin(Wire, I2C_SPEED_STANDARD)) {
    oximetro.setup(0x1F, 4, 2, 100, 411, 4096);
    okMAX30102 = true;
  }

  // 5. RC522 (SPI)
  SPI.begin();
  rfid.PCD_Init();
  byte v = rfid.PCD_ReadRegister(rfid.VersionReg);
  if (v != 0x00 && v != 0xFF) {
    okRFID = true;
  }

  Serial.println("{\"status\":\"FASE1_FREERTOS_OK\",\"sesion\":false}");

  // Creación de tareas concurrentes no bloqueantes (Core 1 para adquisición)
  xTaskCreatePinnedToCore(vTareaRFID,          "TareaRFID",      3072, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(vTareaPulsioximetro, "TareaPulsiox",   3072, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(vTareaCinematica,    "TareaIMU",       3072, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(vTareaTemperatura,   "TareaTemp",      2048, NULL, 1, NULL, 1);
}

void loop() {
  // En FreeRTOS la función loop queda inactiva para ahorrar ciclos de CPU
  vTaskDelete(NULL);
}