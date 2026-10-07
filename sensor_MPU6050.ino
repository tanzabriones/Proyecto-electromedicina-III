#include <Wire.h>
#include <MPU6050_light.h>

// Instancia del sensor MPU6050 sobre el bus Wire
MPU6050 mpu(Wire);

// Control de muestreo sin delay (5 Hz / cada 200 ms para impresión)
unsigned long tPrevIMU = 0;
const unsigned long INTERVALO_IMU = 200;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n=== TEST INDIVIDUAL: MPU6050 ===");

  // Iniciar I2C en pines 21 (SDA) y 22 (SCL)
  Wire.begin(21, 22);

  byte status = mpu.begin();
  if (status != 0) {
    Serial.println("[ERROR] No se pudo inicializar el MPU6050.");
    Serial.println("  -> Revisa alimentacion 3.3V y pines 21/22.");
    Serial.println("  -> Verifica que AD0 este conectado a GND o al aire.");
    while (1); // Detiene ejecución si hay falla física
  }

  Serial.println("[OK] Sensor detectado. Calibrando offsets...");
  Serial.println(">>> NO MUEVAS LA PROTOBOARD DURANTE 3 SEGUNDOS <<<");
  
  mpu.calcOffsets(); // Calcula sesgo de reposo (bias) de acelerómetro y giroscopio
  
  Serial.println("[OK] Calibracion completa. Mostrando lecturas en tiempo real...\n");
}

void loop() {
  // CRÍTICO: mpu.update() debe ejecutarse continuamente en cada ciclo
  // para que el filtro complementario calcule bien la inclinación y ángulos
  mpu.update();

  unsigned long ahora = millis();
  if (ahora - tPrevIMU >= INTERVALO_IMU) {
    tPrevIMU = ahora;

    // Aceleración lineal en g
    float ax = mpu.getAccX();
    float ay = mpu.getAccY();
    float az = mpu.getAccZ();

    // Ángulos calculados en grados (gracias al filtro integrado de la librería)
    float anguloX = mpu.getAngleX();
    float anguloY = mpu.getAngleY();
    float anguloZ = mpu.getAngleZ();

    // Salida estructurada de monitoreo
    Serial.printf("[IMU] Accel (g) -> X: %+.2f | Y: %+.2f | Z: %+.2f  ||  Angulo (deg) -> Roll: %+.1f | Pitch: %+.1f | Yaw: %+.1f\n",
                  ax, ay, az, anguloX, anguloY, anguloZ);
  }
}