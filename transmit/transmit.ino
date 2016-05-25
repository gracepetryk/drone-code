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
#include <SPI.h> // Not actually used but needed to compile

RH_ASK driver;

void setup()
{
    Serial.begin(9600);   // Debugging only
    if (!driver.init())
         Serial.println("init failed");
}

void loop()
{
	String time = String(millis());
	char charbuf[time.length() + 1];
	time.toCharArray(charbuf, time.length() + 1);
    const char *msg = charbuf;
    driver.send((uint8_t *)msg, strlen(msg));
    driver.waitPacketSent();
    delay(10);
}