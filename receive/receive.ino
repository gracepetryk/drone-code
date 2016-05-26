#include <radio_config_Si4460.h>
#include <RadioHead.h>
#include <RH_ASK.h>
#include <RH_NRF24.h>
#include <RH_NRF905.h>
#include <RH_RF22.h>
#include <RH_RF24.h>
#include <RH_RF69.h>
#include <RH_RF95.h>
#include <RH_Serial.h>
#include <RH_TCP.h>
#include <RHCRC.h>
#include <RHDatagram.h>
#include <RHGenericDriver.h>
#include <RHGenericSPI.h>
#include <RHHardwareSPI.h>
#include <RHMesh.h>
#include <RHNRFSPIDriver.h>
#include <RHReliableDatagram.h>
#include <RHRouter.h>
#include <RHSoftwareSPI.h>
#include <RHSPIDriver.h>
#include <RHTcpProtocol.h>
#include <SPI.h> // Not actualy used but needed to compile

RH_ASK driver;

void setup()
{
    Serial.begin(57600); // Debugging only
    if (!driver.init())
         Serial.println("init failed");
}

void loop()
{
    char controlString[64]; 
    uint8_t buf[64];
    uint8_t buflen = sizeof(buf);
    if (driver.recv(buf, &buflen)) // Non-blocking
    {
      int i;
      bool endOfMessage = false;
      // Message with a good checksum received, dump it.
      Serial.print("Message: ");
      for(int i = 0; i < 64; i++)
      {
        if(buf[i] == (uint8_t) 'L')
        {
          endOfMessage = true;
        }
        if (!endOfMessage)
        {
          Serial.print((char)buf[i]);
          controlString[i] = buf[i];
        }
      }
      endOfMessage = false;
      Serial.println(" ");
    }
    delay(5);
    
    
    
}

