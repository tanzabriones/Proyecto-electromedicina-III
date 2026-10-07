#include <Wire.h>
#include <MAX30105.h>

// Instancia del sensor óptico
MAX30105 oximetro;

// Control de muestreo sin delay (20 Hz)
unsigned long tPrevOxi = 0;
const unsigned long INTERVALO_OXI = 50;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n=== TEST INDIVIDUAL: MAX30102 ===");

  Wire.begin(21, 22);
  Wire.setClock(100000); // Forzamos 100 kHz (algunos MAX30102 fallan a 400 kHz)

  if (!oximetro.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println("[ERROR] No se pudo inicializar el MAX30102 en la direccion 0x57.");
    Serial.println("  -> Revisa alimentacion 3.3V y pines 21/22.");
    while (1);
  }

  Serial.println("[OK] MAX30102 detectado. Configurando sensor...");

  // Configuración de LEDs para lectura fisiológica
  byte ledBrightness = 60; // Intensidad: 0 = Apagado, 255 = Máximo (60 es un buen equilibrio)
  byte sampleAverage = 4;  // Promediar 4 muestras adyacentes para reducir ruido
  byte ledMode = 2;        // 1 = Solo Rojo, 2 = Rojo + Infrarrojo, 3 = Rojo + IR + Verde
  int sampleRate = 100;    // Muestreo a 100 Hz
  int pulseWidth = 411;    // Ancho de pulso (411 us otorga resolución de 18 bits en el ADC)
  int adcRange = 4096;     // Escala del convertidor analógico a digital

  oximetro.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);

  Serial.println("[OK] Sensor listo. Pon la yema del dedo suavemente sobre el cristal.");
}

void loop() {
  // .check() actualiza el búfer FIFO interno del sensor continuamente
  oximetro.check();

  unsigned long ahora = millis();
  if (ahora - tPrevOxi >= INTERVALO_OXI) {
    tPrevOxi = ahora;

    // Solo imprime si hay un nuevo dato en el búfer
    if (oximetro.available()) {
      // Obtener lecturas
      uint32_t irValue = oximetro.getFIFOIR();
      uint32_t redValue = oximetro.getFIFORed();
      
      // Avanzar el puntero del búfer a la siguiente lectura
      oximetro.nextSample();

      // Validación simple de contacto físico
      if (irValue < 50000) {
         Serial.println("[PULSO] Sin dedo detectado. Acerca el sensor a la piel.");
      } else {
         // Salida estructurada de monitoreo
         Serial.printf("[PULSO] IR: %u | ROJO: %u\n", irValue, redValue);
      }
    }
  }
}