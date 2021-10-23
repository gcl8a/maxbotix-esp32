#ifndef __MAXBOTIX_H
#define __MAXBOTIX_H

/*
 * Datasheet: https://www.maxbotix.com/documents/LV-MaxSonar-EZ_Datasheet.pdf
 * MCP3002: 
 */

#include <Arduino.h>
#include <SPI.h>

static uint32_t PING_INTERVAL = 250;  //ms

#define ECHO_RECD   0x02
#define UART_RECD   0x04
#define ADC_READ    0x08

#define USE_CTRL_PIN    0x01
#define USE_ECHO        0x02
#define USE_UART        0x04
#define USE_ADC         0x08

class MaxBotix 
{
private:
    uint8_t state = 0;

    uint32_t lastPing = 0;                      //for keeping track of intervals
    uint32_t pingInterval = PING_INTERVAL;    //ms

    uint32_t pulseStart = 0;
    uint32_t pulseEnd = 0;

    String serialString;
public:
    MaxBotix(void);  //ideally, this would take pins as parameters, but just hard-coded for now since we only have one
    void Init(void);
    void Init(uint8_t interfaces);

    //checks to see if it's time for a ping
    uint8_t CheckPingTimer(void);

    //Checks to see if a pulse echo has been registered
    uint16_t CheckEcho(void);

    //Reads the MCP3002 ADC; returns ADC result
    uint16_t ReadMCP3002(void);

    //Checks/reads on the RS-232 interface
    uint16_t ReadASCII(void);

    void MB_ISR(void);
};

extern MaxBotix mb_ez1; 

#endif