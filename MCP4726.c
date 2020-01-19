/*
 * File:   22LC64.c
 * Author: kater
 *
 * Created on 11. ?íjna 2019, 17:52
 */

#include "PICF18LF46K22_ConfigSFR.h"
#include "MCP4726_DA.h"
#include "LCD_Engine4bit.h"
#include "24LC64_PIC.h"


#define AD_GODONE   ADCON0bits.GO_DONE 
#define AD_ENABLE   ADCON0bits.ADON

#define BIT8 256
#define BIT10 1024
#define BIT12 4096

#define SAMPLE_N 4096


#define NumberBuffer_Size 4

char NumberBuffer[NumberBuffer_Size]={};

bit isNullFlag;

char VOLTAGE=0;
char VOLTAGE_MILI=0;

unsigned int SAMPLE=0;

unsigned int getSample();
unsigned int calcSample(unsigned int from,unsigned int to,unsigned int sample);
void calcVoltage(unsigned int sample, unsigned int format);

void Update_LCD();
void Clear_Buffer();
void PutVoltageToLCD(char* label,char volts, char milivolts);

void setProgress();
void setProgressLCD(char progress);
void Init_AD();

void Clear_Buffer(){
    for(char i=0;i<NumberBuffer_Size;i++){
        NumberBuffer[i]=0;
    }
}

void Update_LCD(){ 
    Clear_Buffer();
    calcVoltage(SAMPLE,BIT8);
    
    LCDClearLine(0);
    LCDGoto(0,0);
    PutVoltageToLCD("Voltage: ",VOLTAGE,VOLTAGE_MILI);
}

void PutVoltageToLCD(char* label,char volts, char milivolts){
    LCDPutStr(label);
    convertNumber(volts, NumberBuffer,NumberBuffer_Size);
    LCDPutStr(NumberBuffer);
    LCDPutStr(".");
    if(isNullFlag)LCDPutStr("0");
    
    convertNumber(milivolts, NumberBuffer,NumberBuffer_Size);
    LCDPutStr(NumberBuffer);
    LCDPutStr(" V");
}

unsigned int getSample(){
    AD_GODONE=1;
    NOP();
    while(AD_GODONE);
    NOP();
    return (ADRESH<<8 | ADRESL);
}

unsigned int calcSample(unsigned int from,unsigned int to,unsigned int sample){
    unsigned int result=0x0000;
    float fromResult=(float)sample/from;
    float toResult=to*fromResult;
    
    result=(unsigned int)toResult;
    return result;
}

void calcVoltage(unsigned int sample, unsigned int format){
    isNullFlag=false;
    
    float devider=(float)sample/format;
    float voltage=devider*3.3;
    VOLTAGE=(int)voltage;
    
    if(((int)((voltage-VOLTAGE)*10)) == 0){
        isNullFlag=true;
    }
    VOLTAGE_MILI=(int)((voltage-VOLTAGE)*100);
}

void setProgress(){
    NOP();
    LATD<<=1;
    delay_NOP(8);
    LATD|=1;
    NOP();
}

void setProgressLCD(char progress){
    NOP();
    LCDClearLine(1);
    LCDGoto(0,1);
    for(char i=0;i<progress;i++){
        LCDPutStr("*");
    }
}

void Init_AD(){
    INTCONbits.GIE=1;
    INTCONbits.PEIE=1;

    //Init AD IN
    ANSELAbits.ANSA3=1;
    TRISAbits.RA3=1;
    
    
    ADCON0=0b00001100; //AN3 (RA3) chanel, Disable AD
    ADCON1=0b00000000; //Vref + to Vdd, Vref - to Vss
    ADCON2=0b10101101; //ADFM=1 , ACQT 12 TAD , ADCS Fosc/16 (TAD=1us)

}

void InitDevice(){
    OSCCON=0b01111100; //osc setting, 16 MHz, internal by FOSCH
    //LEDs on port for show result data
    ANSELD=0;
    TRISD=0;
    PORTD=0x00;
 
    LCD_Initialize();
    __delay_ms(10);
    Init_MCP4726();
    __delay_ms(10);
    Init_24LC64();
    __delay_ms(10);
    Init_AD();
    delay_ms(10);
}

void TestDevice(){
    LCDPutStr("Preparing...");
    Clear_Buffer();
    
    delay_ms(1000);
    LCDClearLine(0);
    LCDGoto(5,0);
    LCDPutStr("READY!");
    delay_ms(2500);
    
    LCD_Clear();
}

void ClearDevice(){
    ADRESH=ADRESL=0x00;
}


void main(void) {
    InitDevice();
    TestDevice();
    ClearDevice();
    
    AD_ENABLE=1;
    
    char devider=2;
    
    LCDPutStr("Measuring...");
    NOP();
    for(char h=0;h<H_MAX;h++){
        for(char l=0;l<L_MAX;l++){
           SAMPLE=getSample();
           NOP(); 
           WriteTo_24LC64(h,l,(char)calcSample(BIT10,BIT8,SAMPLE));
        }
        if( (h % devider == 0) && h!=0)setProgressLCD(h/devider);
    }
    NOP();
    LCD_Clear();
    LCDGoto(5,10);
    LCDPutStr("DONE!");
    LCDGoto(0,1);
    LCDPutStr("Preparing...");
    delay_ms(2500);
    LCD_Clear();
    
    for(char h=0;h<H_MAX;h++){
        for(char l=0;l<L_MAX;l++){
           SAMPLE=ReadFrom_24LC64(h,l);
           Set_VoltageAsData_MCP4726(calcSample(BIT8,BIT12,SAMPLE));
           if( l % 10 == 0)Update_LCD();
        }
        if( (h % devider == 0) && h!=0)setProgressLCD(h/devider);
    }
    AD_ENABLE=0;
    LCD_Clear();
    Set_VoltageAsData_MCP4726(0);
    LCDPutStr("TASK FINISH...");
    
    while(1){
        asm("NOP");
    }
}

/*void interrupt IRS(void){
}*/
