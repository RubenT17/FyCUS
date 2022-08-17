#include <SPI.h>
#include <LoRa.h>


#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 14
#define DIO0 26

//433E6
//866E6
//915E6
#define BAND 433E6


void setup() { 
  Serial.begin(115200);
  while(!Serial);
  
  SPI.begin(SCK, MISO, MOSI, SS);
  LoRa.setPins(SS, RST, DIO0);

  if (!LoRa.begin(BAND)) 
  {
    while (1)
    {
      Serial.println(F("*** ERROR LoRa ***"));
      delay(1000);
    }
  }
  else
  {                                  
    Serial.println(F("LoRa INITIALIZATION SUCCESSFUL"));
    LoRa.setSpreadingFactor(12);
    LoRa.setCodingRate4(8);
    LoRa.enableCrc();
    LoRa.setTxPower(20);
  }
}

void loop() {
  if (LoRa.parsePacket()) 
  {
    Serial.print("Received packet:  ");
      Serial.println(LoRa.readString());
    Serial.print("\t - RSSI: ");
      Serial.print(LoRa.packetRssi());
      Serial.println(" dBm");
    Serial.print("\t - SNR:  ");
      Serial.print(LoRa.packetSnr());
      Serial.println(" dB");
    Serial.print("\t - Freq. Error:  ");
      Serial.print(LoRa.packetFrequencyError());
      Serial.println(" Hz");
      Serial.println("\n");
  }
}

  /* COSAS QUE FALTAN:
   *  - Dormir al ESP32 hasta que haya un paquete disponible y no gastar energía haciendo polling
   */
