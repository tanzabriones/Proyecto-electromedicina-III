#include <SPI.h>
#include <MFRC522.h>

// Definición de pines para ESP32
#define RST_PIN   22
#define SS_PIN    5

MFRC522 rfid(SS_PIN, RST_PIN);

void setup() {
  Serial.begin(115200);
  SPI.begin();          // Inicia el bus SPI
  rfid.PCD_Init();      // Inicializa el lector MFRC522

  Serial.println("\n--- Lector RFID-RC522 Listo ---");
  Serial.println("Acerca una tarjeta o llavero...");
}

void loop() {
  // Revisa si hay una nueva tarjeta presente
  if (!rfid.PICC_IsNewCardPresent()) {
    return;
  }

  // Intenta leer el número de serie (UID) de la tarjeta
  if (!rfid.PICC_ReadCardSerial()) {
    return;
  }

  // Imprime el UID en formato Hexadecimal
  Serial.print("UID Detectado: ");
  String uidString = "";
  
  for (byte i = 0; i < rfid.uid.size; i++) {
    // Agrega un 0 a la izquierda si el valor es menor a 0x10 para formato estándar
    if (rfid.uid.uidByte[i] < 0x10) {
      Serial.print(" 0");
      uidString += " 0";
    } else {
      Serial.print(" ");
      uidString += " ";
    }
    Serial.print(rfid.uid.uidByte[i], HEX);
    uidString += String(rfid.uid.uidByte[i], HEX);
  }
  Serial.println();

  // Tipo de tarjeta detectada
  MFRC522::PICC_Type piccType = rfid.PICC_GetType(rfid.uid.sak);
  Serial.print("Tipo de tarjeta: ");
  Serial.println(rfid.PICC_GetTypeName(piccType));

  // Detiene la comunicación con la tarjeta actual para permitir nuevas lecturas
  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();

  delay(1000); // Pausa breve para evitar lecturas duplicadas continuas
}