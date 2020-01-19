/*
 * File:   ad_measure.c
 * Author: kater
 *
 * Created on 14. ledna 2020, 20:50
 */

#include "PICF18LF46K22_ConfigSFR.h"
#include "MCP4726_DA.h"

#define AD_GODONE  ADCON0bits.GO_DONE 
#define AD_ENABLE   ADCON0bits.ADON
#define BUFFMEM 128

unsigned int indexer;
unsigned int Buffer[BUFFMEM];

unsigned int getSample(){
    AD_GODONE=1;
    NOP();
    while(AD_GODONE);
    NOP();
    return (ADRESH<<8 | ADRESL);
}

void ClearBuff(){
    for(int i=0; i<BUFFMEM; i++)Buffer[i]=0x00;
    indexer=0;
}

bit isBufferFull(){
    if(indexer>=BUFFMEM)return true;
    else false;
}

void setProgress(){
    NOP();
    LATD<<=1;
    delay_NOP(8);
    LATD|=1;
    NOP();
}


void InitDevice(){
    OSCCON=0b01111100; //osc setting, 16 MHz, internal by FOSCH
    INTCON=0b11000000; //GIE, PIE
    //Init LEDs
    ANSELD=0;
    TRISD=0;

    //Init AD IN
    ANSELAbits.ANSA3=1;
    TRISAbits.RA3=1;
    
    
    ADCON0=0b00001100; //AN3 (RA3) chanel, Disable AD
    ADCON1=0b00000000; //Vref + to Vdd, Vref - to Vss
    ADCON2=0b10101101; //ADFM=1 , ACQT 12 TAD , ADCS Fosc/16 (TAD=1us)
    
    //result ADRESH, ADRESL   
}

void ClearDevice(){
    ADRESH=ADRESL=0;
    PORTD=0;
    ClearBuff();
}

void TestDevice(){
    PORTD=0xFF;
    __delay_ms(500);
    PORTD=0;
}

void main(void) {
    InitDevice();
    ClearDevice();
    TestDevice();
    
    Init_MCP4726();
    AD_ENABLE=1;
    
    char devider=BUFFMEM/8;
    unsigned int sample=0;
    
    for(int i=0;i<BUFFMEM;i++){
        if( i % devider == 0)setProgress();
        sample=getSample();
        NOP();
        
        //Set_VoltageAsData_MCP4726(sample);
        Buffer[i]=sample;
        delay_ms(100);
    }
    
    PORTD=0xFF;
    delay_ms(1000);
    PORTD=0x00;
    while(1);
}
