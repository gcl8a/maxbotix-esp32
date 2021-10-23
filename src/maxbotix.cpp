#include "maxbotix.h"

#define MB_CTRL 2       //for pinging -- not a great choice since this can hamper uploading
#define PULSE_PIN 35    //for reading pulse

#define MB_WINDOW_DUR 50    //ms

MaxBotix mb_ez1;

uint8_t MaxBotix::checkPingTimer(void)
{
    //check if we're ready to ping
    if(millis() - lastPing >= pingInterval)
    {
        pulseEnd = pulseStart = 0;

        //clear out any leftover states
        state = 0;

        lastPing = millis(); //not perfectly on schedule, but safer and close enough

        digitalWrite(MB_CTRL, HIGH); //commands a ping; leave high for the duration
        delayMicroseconds(30); //datasheet says hold HIGH for >20us
        digitalWrite(MB_CTRL, LOW); //unclear if pin has to stay HIGH
        
        // Serial.print('\n');
        // Serial.print(lastPing);
        // Serial.print("\tping\t");
    }

    return state;
}

uint16_t MaxBotix::checkEcho(void)
{
    uint16_t echoLength = 0;
    if(state & ECHO_RECD)
    {
        echoLength = pulseEnd - pulseStart;
        state &= ~ECHO_RECD;
    }

    return echoLength;
}

void ISR_MaxBotix(void)
{
    mb_ez1.MB_ISR();
}

MaxBotix::MaxBotix(void) {}

void MaxBotix::init(void)
{
    init(USE_ADC | USE_UART | USE_ECHO | USE_CTRL_PIN);
}

void MaxBotix::init(uint8_t interfaces)
{
    if(interfaces & USE_ECHO)
    {
        // assert ECHO pin is an input
        pinMode(PULSE_PIN, INPUT);
        attachInterrupt(PULSE_PIN, ISR_MaxBotix, CHANGE);
    }

    if(interfaces & USE_UART)
    {
        //Serial2 is used for RS-232 format
        Serial2.begin(9600);
        Serial2.setRxInvert(true); // MaxBotix uses INVERTED logic levels
    }

    if(interfaces & USE_ADC) //uses the MCP3002, not an onboard ADC
    {
        //SPI to talk to the MCP3002
        SPI.begin(); //defaults to VPSI: SCK, MISO, MOSI, SS; see above
        pinMode(SS, OUTPUT); //need to set the CS to OUTPUT
    }

    if(interfaces & USE_CTRL_PIN)
    {
        //control pin for commanding pings
        pinMode(MB_CTRL, OUTPUT);
    }
}

uint16_t MaxBotix::readMCP3002(void)
{
  //this will command the MCP to take a reading on ADC1; the datasheet has details
  uint16_t cmdByte = 0x6800; 

  //start the SPI session
  SPISettings spiSettings; //defaults to (1000000, MSBFIRST, SPI_MODE0), which is what we want
  SPI.beginTransaction(spiSettings); 
  
  //open communication with the MCP3002
  digitalWrite(SS, LOW); 

  //this line both sends the command to read AND retrieves the result
  //the leading bits are indeterminate and need to be stripped off
  uint16_t ADCvalue = SPI.transfer16(cmdByte) & 0x02ff;
  
  //end communication
  digitalWrite(SS, HIGH); 
  
  //close the SPI session
  SPI.endTransaction(); 

  return ADCvalue;
}

uint16_t MaxBotix::readASCII(void)
{
  while(Serial2.available())
  {
    char c = Serial2.read();
    
    //comment these out to suppress raw uart debugging
    // Serial.print(c, HEX);
    // Serial.print(' ');

    if(c != 'R') serialString += c;

    if(c == 0xD) 
    {
      uint16_t result = serialString.toInt();
      serialString = "";
      return result;
    }
  }

  return 0;
}

//ISR for echo pin
void MaxBotix::MB_ISR(void)
{
    if(digitalRead(PULSE_PIN))  //transitioned to HIGH
    {
        pulseStart = micros();
    }

    else                        //transitioned to LOW
    {
        pulseEnd = micros();
        state |= ECHO_RECD;
    } 
}
