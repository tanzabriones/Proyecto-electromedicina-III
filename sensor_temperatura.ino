#include <OneWire.h>
#include <DallasTemperature.h>

// Pin asignado en la arquitectura del proyecto
#define PIN_DS18B20 15

// Instancias del bus One-Wire
OneWire oneWire(PIN_DS18B20);
DallasTemperature sensorTemp(&oneWire);

// Control de tiempo no bloqueante (1 Hz)
unsigned long tPrevTemp = 0;
const unsigned long INT_TEMP = 1000;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n==================================================");
  Serial.println("   UNIVERSIDAD DE VALPARAÍSO - ELECTROMEDICINA III");
  Serial.println("           PRUEBA EXCLUSIVA: DS18B20");
  Serial.println("==================================================");

  // Activa el pull-up interno del ESP32 para reforzar la línea si el externo falla
  pinMode(PIN_DS18B20, INPUT_PULLUP);

  // Inicializar sensor
  sensorTemp.begin();
  
  // Comprobar si el bus detecta físicamente el sensor
  int dispositivos = sensorTemp.getDeviceCount();
  Serial.printf("Dispositivos detectados en el bus: %d\n", dispositivos);

  if (dispositivos == 0) {
    Serial.println("[ERROR] No se encontro el DS18B20 en el GPIO 15.");
    Serial.println(" -> Revisa la resistencia pull-up de 4.7k entre GPIO 15 y 3.3V.");
    Serial.println(" -> Verifica que GND y 3.3V esten bien insertados en la protoboard.");
  } else {
    Serial.println("[OK] Sensor DS18B20 encontrado y listo.");
    // Modo asíncrono para no congelar el microcontrolador
    sensorTemp.setWaitForConversion(false);
    sensorTemp.requestTemperatures(); 
  }
  Serial.println("==================================================\n");
}

void loop() {
  unsigned long ahora = millis();

  if (ahora - tPrevTemp >= INT_TEMP) {
    tPrevTemp = ahora;

    float tempC = sensorTemp.getTempCByIndex(0);

    // Validación según el Modelo de Medición de la guía
    bool valida = (tempC != DEVICE_DISCONNECTED_C && tempC > 10.0 && tempC < 45.0);

    if (tempC == DEVICE_DISCONNECTED_C) {
      Serial.printf("{\"sensor\":\"DS18B20\", \"status\":\"DISCONNECTED\", \"ts\":%lu}\n", ahora);
    } else {
      // Salida estructurada con contexto: valor, unidad, validez y marca de tiempo
      Serial.printf("{\"sensor\":\"DS18B20\", \"temp\":%.2f, \"unit\":\"C\", \"valid\":%s, \"ts\":%lu}\n",
                    tempC, valida ? "true" : "false", ahora);
    }

    // Solicita la lectura asíncrona para el siguiente segundo
    sensorTemp.requestTemperatures();
  }
}