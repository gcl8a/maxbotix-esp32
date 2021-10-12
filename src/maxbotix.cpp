#include "maxbotix.h"

#define MB_CTRL 2   //for pinging
#define PULSE_PIN 35 //for reading pulse

#define MB_WINDOW_DUR 50    //ms

MaxBotix mb_ez1;

SPISettings spiSettings; //defaults to (1000000, MSBFIRST, SPI_MODE0), which is what we want

uint8_t MaxBotix::CheckSonar(void)
{
    if(!(state & PINGING)) //
    {
        //check if we're ready to take a reading
        if(millis() - lastPing >= pingInterval)
        {
            state = PINGING; //reset all the other flags

            adcValue = 0;
            rsDistance = 0;
            pulseEnd = pulseStart = 0;

            lastPing = millis(); //not perfectly on schedule, but safer and close enough
            digitalWrite(MB_CTRL, HIGH); //commands a ping; leave high for the duration

            // Serial.print('\n');
            // Serial.print(lastPing);
            // Serial.print("\tping\t");
        }
    }

    else
    {
        uint16_t rsDist = ReadASCII();
        if(rsDist) 
        {
            rsDistance = rsDist;
            state |= UART_RECD;
        }

        if(millis() - lastPing > MB_WINDOW_DUR) 
        {
            adcValue = ReadMCP3002();
            state |= ADC_READ;
            state &= ~PINGING;
        }
    }

    return state;
}

uint8_t MaxBotix::Print(void)
{
    //EDIT THIS LINE: convert pulseLength to a distance
    float distancePulse = 0;

    //and print it out
    Serial.print(pulseEnd - pulseStart);
    Serial.print('\t');
    Serial.print(distancePulse);
    Serial.print('\t');

    //EDIT THESE LINES: convert the ADC reading to voltage and then voltage to distance
    float voltage = 0;
    float distanceADC = 0;

    //and print them all out
    Serial.print(adcValue);
    Serial.print('\t');
    Serial.print(voltage);
    Serial.print('\t');
    Serial.print(distanceADC);
    Serial.print('\t');

    Serial.print(rsDistance);
    Serial.print('\n');

    return state = 0; //reset the state after we print
}

void ISR_MaxBotix(void)
{
    mb_ez1.MB_ISR();
}

MaxBotix::MaxBotix(void) {}

void MaxBotix::Init(void)
{
    //Serial2 is used for RS-232 format
    Serial2.begin(9600);
    Serial2.setRxInvert(true); // MaxBotix uses INVERTED logic levels

    //SPI to talk to the MCP3002
    SPI.begin(); //defaults to VPSI: SCK, MISO, MOSI, SS; see above
    pinMode(SS, OUTPUT); //need to set the CS to OUTPUT

    //control pin for commanding pings
    pinMode(MB_CTRL, OUTPUT);

    // assert ECHO pin is an input
    pinMode(PULSE_PIN, INPUT);
    attachInterrupt(PULSE_PIN, ISR_MaxBotix, CHANGE);
}

void MaxBotix::Init(uint8_t interfaces)
{
    config = interfaces;
    
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

uint32_t MaxBotix::ReadMCP3002(void)
{
  //this will command the MCP to take a reading on ADC1; the datasheet has details
  uint16_t cmdByte = 0x6800; 

  //start the SPI session
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

uint32_t MaxBotix::ReadASCII(void)
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
      uint32_t result = serialString.toInt();
      serialString = "";
      return result;
    }
  }

  return 0;
}

//ISR for echo pin
void MaxBotix::MB_ISR(void)
{
    if(state & PINGING)
    {
        if(digitalRead(PULSE_PIN))  //transitioned to HIGH
        {
            pulseStart = micros();
            digitalWrite(MB_CTRL, LOW); //stop pinging
        }

        else                        //transitioned to LOW
        {
            pulseEnd = micros();
            state |= ECHO_RECD;
        } 
    }
    
    else {} //really should handle the error
}
