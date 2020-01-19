/*
 * File:   22LC64.c
 * Author: kater
 *
 * Created on 11. ?íjna 2019, 17:52
 */

#include "PICF18LF46K22_ConfigSFR.h"
#include "MCP4726_DA.h"
#include "LCD_Engine4bit.h"


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

char CURRENT_mA=0;

unsigned int SAMPLE=0;

unsigned int getSample();
unsigned int calcSample(unsigned int from,unsigned int to,unsigned int sample);
void calcVoltage(unsigned int sample, unsigned int format);
void calcCurrent(unsigned int sample, unsigned int format);

void Update_LCD();
void Clear_Buffer();
void PutVoltageToLCD(char* label,char volts, char milivolts);
void PutCurrentToLCD(char* label,char current_mA);

void Init_AD();

void Clear_Buffer(){
    for(char i=0;i<NumberBuffer_Size;i++){
        NumberBuffer[i]=0;
    }
}

void Update_LCD(){ 
    Clear_Buffer();
    
    calcVoltage(SAMPLE,BIT10);
    LCDClearLine(0);
    LCDGoto(0,0);
    PutVoltageToLCD("Voltage: ",VOLTAGE,VOLTAGE_MILI);
    
    calcCurrent(SAMPLE,BIT10);
    LCDClearLine(1);
    LCDGoto(0,1);
    PutCurrentToLCD("Current: ",CURRENT_mA);
    
    delay_ms(10);
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

void PutCurrentToLCD(char* label,char current_mA){
    LCDPutStr(label);
    convertNumber(current_mA, NumberBuffer,NumberBuffer_Size);
    LCDPutStr(NumberBuffer);
    
    if(isNullFlag)LCDPutStr(" uA");
    else LCDPutStr(" mA");
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

void calcCurrent(unsigned int sample, unsigned int format){
    isNullFlag=false;
    
    unsigned int resistance=10;
    float devider=(float)sample/format;
    float voltage=devider*3.3;
    float current=voltage/resistance;
    if((char)(current*10) == 0){
        isNullFlag=true;
    }
    CURRENT_mA=(char)(current*100);
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
    Init_AD();
    delay_ms(10);
}

void TestDevice(){
    LCDPutStr("Preparing...");
    Clear_Buffer();
    
    delay_ms(1000);
    LCDClearLine(0);
    LCDGoto(3,0);
    LCDPutStr("-VOLTMETR-");
    LCDGoto(3,1);
    LCDPutStr("-GENERATOR-");
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
    
    char i=0;
    
    AD_ENABLE=1;
    NOP();
    while(1){
        i++;
        SAMPLE=getSample();
        NOP(); 
        Set_VoltageAsData_MCP4726(calcSample(BIT10,BIT12,SAMPLE));     
        if( i % 10 == 0){
            Update_LCD();
            i=0;
        }
        delay_ms(25);
    }
    AD_ENABLE=0;
    LCD_Clear();  
    while(1){
        asm("NOP");
    }
}

/*void interrupt IRS(void){
}*/
