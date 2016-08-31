/********************************************************************
 FileName:		main.c
 Dependencies:	See INCLUDES section
 Processor:		PIC18 or PIC24 USB Microcontrollers
 Hardware:		The code is natively intended to be used on the following
 				hardware platforms: PICDEM?FS USB Demo Board,
 				PIC18F87J50 FS USB Plug-In Module, or
 				Explorer 16 + PIC24 USB PIM.  The firmware may be
 				modified for use on other USB platforms by editing the
 				HardwareProfile.h file.
 Complier:  	Microchip C18 (for PIC18) or C30 (for PIC24)
 Company:		Microchip Technology, Inc.

 Software License Agreement:

 The software supplied herewith by Microchip Technology Incorporated
 (the ?ompany? for its PIC?Microcontroller is intended and
 supplied to you, the Company? customer, for use solely and
 exclusively on Microchip PIC Microcontroller products. The
 software is owned by the Company and/or its supplier, and is
 protected under applicable copyright laws. All rights are reserved.
 Any use in violation of the foregoing restrictions may subject the
 user to criminal sanctions under applicable laws, as well as to
 civil liability for the breach of the terms and conditions of this
 license.

 THIS SOFTWARE IS PROVIDED IN AN ?S IS?CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.

********************************************************************
 File Description:

 Change History:
  Rev   Date         Description
  1.0   11/19/2004   Initial release
  2.1   02/26/2007   Updated for simplicity and to use common
                     coding style
********************************************************************/

/** INCLUDES *******************************************************/
#include "Compiler.h"
#include "HardwareProfile.h"
#include "GenericTypeDefs.h"
#include "USB/usb_device.h"
#include "USB/usb.h"
#include "USB/usb_function_generic.h"
#include "usb_config.h"
#include <p24FJ256GB106.h>
#include <xc.h>
#include <i2c.h>

// PIC24FJ256GB106 Configuration Bit Settings

// CONFIG3
#pragma config WPFP = WPFP511           // Write Protection Flash Page Segment Boundary (Highest Page (same as page 170))
#pragma config WPDIS = WPDIS            // Segment Write Protection Disable bit (Segmented code protection disabled)
#pragma config WPCFG = WPCFGDIS         // Configuration Word Code Page Protection Select bit (Last page(at the top of program memory) and Flash configuration words are not protected)
#pragma config WPEND = WPENDMEM         // Segment Write Protection End Page Select bit (Write Protect from WPFP to the last page of memory)

// CONFIG2
#pragma config POSCMOD = HS             // Primary Oscillator Select (HS oscillator mode selected)
#pragma config DISUVREG = OFF           // Internal USB 3.3V Regulator Disable bit (Regulator is disabled)
#pragma config IOL1WAY = ON             // IOLOCK One-Way Set Enable bit (Write RP Registers Once)
#pragma config OSCIOFNC = OFF           // Primary Oscillator Output Function (OSCO functions as CLKO (FOSC/2))
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor (Both Clock Switching and Fail-safe Clock Monitor are disabled)
#pragma config FNOSC = PRIPLL           // Oscillator Select (Primary oscillator (XT, HS, EC) with PLL module (XTPLL,HSPLL, ECPLL))
#pragma config PLL_96MHZ = ON           // 96MHz PLL Disable (Enabled)
#pragma config PLLDIV = DIV6            // USB 96 MHz PLL Prescaler Select bits (Oscillator input divided by 6 (24MHz input))
#pragma config IESO = OFF               // Internal External Switch Over Mode (IESO mode (Two-speed start-up)disabled)

// CONFIG1
#pragma config WDTPS = PS32768          // Watchdog Timer Postscaler (1:32,768)
#pragma config FWPSA = PR128            // WDT Prescaler (Prescaler ratio of 1:128)
#pragma config WINDIS = OFF             // Watchdog Timer Window (Standard Watchdog Timer enabled,(Windowed-mode is disabled))
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (Watchdog Timer is disabled)
#pragma config ICS = PGx1               // Comm Channel Select (Emulator functions are shared with PGEC1/PGED1)
#pragma config GWRP = OFF               // General Code Segment Write Protect (Writes to program memory are allowed)
#pragma config GCP = OFF                // General Code Segment Code Protect (Code protection is disabled)
#pragma config JTAGEN = OFF             // JTAG Port Enable (JTAG port is disabled)




// timer


#define STOP_TIMER_IN_IDLE_MODE     0x2000
#define TIMER_SOURCE_INTERNAL       0x0000
#define TIMER_ON                    0x8000
#define TIMER_OFF 					0x7FFF // and operation
#define GATED_TIME_DISABLED         0x0000
#define TIMER_16BIT_MODE            0x0000
#define TIMER_PRESCALER_1           0x0000
#define TIMER_PRESCALER_8           0x0010
#define TIMER_PRESCALER_64          0x0020
#define TIMER_PRESCALER_256         0x0030
#define TIMER_INTERRUPT_PRIORITY    0x1000



//********************************************************************************/


#define STOP_TIMER_IN_IDLE_MODE     0x2000
#define TIMER_SOURCE_INTERNAL       0x0000
#define TIMER_ON                    0x8000
#define TIMER_OFF 					0x7FFF // and operation
#define GATED_TIME_DISABLED         0x0000
#define TIMER_16BIT_MODE            0x0000
#define TIMER_PRESCALER_1           0x0000
#define TIMER_PRESCALER_8           0x0010
#define TIMER_PRESCALER_64          0x0020
#define TIMER_PRESCALER_256         0x0030
#define TIMER_INTERRUPT_PRIORITY    0x1000

//User Defined values
#define DELAY 24000

#define DATAPREFIXLENGTH 4
#define LEDROWS 12
#define LEDCOLS 22
#define LEDSIZE LEDROWS * LEDCOLS // 264
#define FORCESENSOR 4
#define TOUCHSENSOR 3
#define DATASUFFIX 1
#define DATALENGTH DATAPREFIXLENGTH+LEDSIZE+FORCESENSOR+TOUCHSENSOR+DATASUFFIX //4+264+4+3+1 = 276
#define writeAddr(addr) ( (addr << 1) & 0xFE)
#define readAddr(addr) ( (addr << 1) | 0x01)
unsigned int data[DATALENGTH];
unsigned int OnData[LEDSIZE];
unsigned int OffData[LEDSIZE+1];

//********************************************************************************/

//int cnt;
int i=0;
int bufferFlag = 0;
int count = 0;
BOOL on = FALSE;

volatile DWORD    tick;

unsigned char OUTPacket[64];	//User application buffer for receiving and holding OUT packets sent from the host
unsigned char INPacket[64];		//User application buffer for sending IN packets to the host
#pragma udata
BOOL blinkStatusValid;
USB_HANDLE USBGenericOutHandle;
USB_HANDLE USBGenericInHandle;
#pragma udata

#define Fosc	(24000000)
#define Fcy	(Fosc/2*6)	// no PLL
#define Fsck	100000
#define I2C_BRG	((Fcy/Fsck-Fcy/10000000)-1)
#define I2C_ON  0x8000



/** PRIVATE PROTOTYPES *********************************************/


void TickInit( void );
//User defined functions --
void SetPorts(void);
void CollectProximity(void);
void CollectForce(void);
void CollectTouch(void);
void SetHaptic(unsigned char haptic);
void delay(unsigned int ticks);
void intitADCL(void);
void SendData();
void CalibrateTouch(void);
#pragma code



int main(void)

{

    SetPorts();
    PORTFbits.RF3 = 0;
//    I2C2BRG = 236; //for Fscl = 100k, and Fcy = 24M,
//    I2C2CONbits.I2CEN = 1;

    OpenI2C2(I2C_ON, I2C_BRG);
   
    USBGenericOutHandle = 0;
    USBGenericInHandle = 0;
    USBDeviceInit();
    USBDeviceAttach();
    int i = 0;

    data[0] = 0;
    data[1] = 0xffff;
    data[2] = 0;
    data[3] = 0xffff;
    data[DATAPREFIXLENGTH+LEDSIZE+FORCESENSOR+TOUCHSENSOR] = 5;//Device ID
    //CalibrateTouch();
    while(1){
        PORTFbits.RF3 = 0;
        CollectProximity();
        CollectTouch();
        CollectForce();
        PORTFbits.RF3 = 1;
        SendData();
//        SetHaptic(3);
    }
}//end main

void CollectTouch(){
    unsigned char TouchSlaveAddress  = 0b00000011;
    unsigned char buffer[3];
    StartI2C2();	//Send the Start Bit
    IdleI2C2();		//Wait to complete
    MasterWriteI2C2( readAddr(TouchSlaveAddress) );
    IdleI2C2();
    buffer[0] = MasterReadI2C2();
    IdleI2C2();
    AckI2C2();
    IdleI2C2();
    while(I2C2CONbits.ACKEN);
    buffer[1] = MasterReadI2C2();
    IdleI2C2();
    AckI2C2();
    IdleI2C2();
    while(I2C2CONbits.ACKEN);
    buffer[2] = MasterReadI2C2();
    IdleI2C2();
    NotAckI2C2();
    IdleI2C2();
    while(I2C2CONbits.ACKEN);
    IdleI2C2();
    StopI2C2();	//Send the Stop condition
    IdleI2C2();	//Wait to complete
    data[DATAPREFIXLENGTH+LEDSIZE+FORCESENSOR] = buffer[0];
    data[DATAPREFIXLENGTH+LEDSIZE+FORCESENSOR+1] = buffer[1];
    data[DATAPREFIXLENGTH+LEDSIZE+FORCESENSOR+2] = buffer[2];
}

void CalibrateTouch(){
    //I2C 2
    unsigned char TouchSlaveAddress  = 0b00000011;
    
    StartI2C2();	//Send the Start Bit
    IdleI2C2();		//Wait to complete

    MasterWriteI2C2( writeAddr(TouchSlaveAddress) );
    IdleI2C2();
    MasterWriteI2C2( 'w' );  // Write sends just one byte
    IdleI2C2();

    StopI2C2();	//Send the Stop condition
    IdleI2C2();	//Wait to complete
}

void SetHaptic(unsigned char haptic){
    //I2C 2
    unsigned char HapticSlaveAddress = 0b00000010;
    StartI2C2();	//Send the Start Bit
    IdleI2C2();		//Wait to complete

    MasterWriteI2C2( writeAddr(HapticSlaveAddress) );
    IdleI2C2();
    MasterWriteI2C2( haptic );  // Write sends just one byte
    IdleI2C2();

    StopI2C2();	//Send the Stop condition
    IdleI2C2();	//Wait to complete
}

void CollectForce(){
    //I2C 2
    unsigned char ForceSlaveAddress = 0b00000010;
    unsigned char buffer[4];
    StartI2C2();	//Send the Start Bit
    IdleI2C2();		//Wait to complete
    MasterWriteI2C2( readAddr(ForceSlaveAddress) );
    IdleI2C2();
    buffer[0] = MasterReadI2C2();
    IdleI2C2();
    AckI2C2();
    IdleI2C2();
    while(I2C2CONbits.ACKEN);
    buffer[1] = MasterReadI2C2();
    IdleI2C2();
    AckI2C2();
    IdleI2C2();
    while(I2C2CONbits.ACKEN);
    buffer[2] = MasterReadI2C2();
    IdleI2C2();
    AckI2C2();
    IdleI2C2();
    while(I2C2CONbits.ACKEN);
    buffer[3] = MasterReadI2C2();
    IdleI2C2();
    NotAckI2C2();
    IdleI2C2();
    while(I2C2CONbits.ACKEN);
    IdleI2C2();
    StopI2C2();	//Send the Stop condition
    IdleI2C2();	//Wait to complete
    data[DATAPREFIXLENGTH+LEDSIZE] = buffer[0];
    data[DATAPREFIXLENGTH+LEDSIZE+1] = buffer[1];
    data[DATAPREFIXLENGTH+LEDSIZE+2] = buffer[2];
    data[DATAPREFIXLENGTH+LEDSIZE+3] = buffer[3];

//    SetHaptic(buffer[1]);
}

void CollectProximity(){


    int row = 0;
    int col = 0;
    int t_input = 0;
    int idx = 0;
    AD1CHSbits.CH0SA = 15;// CHANNEL 0 POSITIVE INPUTA (MUXA) = AN15
    delay(60);
    IFS0bits.AD1IF = 0;
    AD1CON1bits.ASAM = 1;
    while (!IFS0bits.AD1IF){};
    AD1CON1bits.ASAM=0;
    OffData[0] = ADC1BUF0;
//                t_input-= ADC1BUF0;

    IFS0bits.AD1IF = 0;
    AD1CON1bits.ASAM = 1;
    while (!IFS0bits.AD1IF){};
    AD1CON1bits.ASAM=0;
    OffData[0]+= ADC1BUF0;
//                t_input-= ADC1BUF0;

    IFS0bits.AD1IF = 0;
    AD1CON1bits.ASAM = 1;
    while (!IFS0bits.AD1IF){};
    AD1CON1bits.ASAM=0;
    OffData[0]+= ADC1BUF0;
//                t_input-= ADC1BUF0;

    IFS0bits.AD1IF = 0;
    AD1CON1bits.ASAM = 1;
    while (!IFS0bits.AD1IF){};
    AD1CON1bits.ASAM=0;
    OffData[0]+= ADC1BUF0;



    for(col = 0; col<LEDCOLS; col++){
        if(col == 0){
            PORTEbits.RE0 = 0;
        }
        else{
            PORTEbits.RE0 = 1;
            if(col == 1)
                delay(3);
        }
        PORTEbits.RE1=1;
        delay(3);
        PORTEbits.RE1=0;
        for(row = 0; row<LEDROWS*2; row++){
            idx = (row>>1)*LEDCOLS+col;
            if(row == 0){
                PORTEbits.RE2 = 1;
            }
            else{
                PORTEbits.RE2 = 0;
                if(row==1)
                    delay(3);
            }
            PORTEbits.RE3=1;
            delay(60);

            PORTEbits.RE3=0;

            if((row&1) == 0){ // When LED on


                IFS0bits.AD1IF = 0;
                AD1CON1bits.ASAM = 1;
                while (!IFS0bits.AD1IF){};
                AD1CON1bits.ASAM=0;
                OnData[idx] = ADC1BUF0;
//                t_input = ADC1BUF0;

                IFS0bits.AD1IF = 0;
                AD1CON1bits.ASAM = 1;
                while (!IFS0bits.AD1IF){};
                AD1CON1bits.ASAM=0;
                OnData[idx]+= ADC1BUF0;
//                t_input+= ADC1BUF0;

                IFS0bits.AD1IF = 0;
                AD1CON1bits.ASAM = 1;
                while (!IFS0bits.AD1IF){};
                AD1CON1bits.ASAM=0;
                OnData[idx]+= ADC1BUF0;
//                t_input+= ADC1BUF0;

                IFS0bits.AD1IF = 0;
                AD1CON1bits.ASAM = 1;
                while (!IFS0bits.AD1IF){};
                AD1CON1bits.ASAM=0;
                OnData[idx]+= ADC1BUF0;
//                t_input+= ADC1BUF0;

            }
            else{ // When LED off
                IFS0bits.AD1IF = 0;
                AD1CON1bits.ASAM = 1;
                while (!IFS0bits.AD1IF){};
                AD1CON1bits.ASAM=0;
                OffData[idx+1] = ADC1BUF0;
//                t_input-= ADC1BUF0;

                IFS0bits.AD1IF = 0;
                AD1CON1bits.ASAM = 1;
                while (!IFS0bits.AD1IF){};
                AD1CON1bits.ASAM=0;
                OffData[idx+1]+= ADC1BUF0;
//                t_input-= ADC1BUF0;

                IFS0bits.AD1IF = 0;
                AD1CON1bits.ASAM = 1;
                while (!IFS0bits.AD1IF){};
                AD1CON1bits.ASAM=0;
                OffData[idx+1]+= ADC1BUF0;
//                t_input-= ADC1BUF0;

                IFS0bits.AD1IF = 0;
                AD1CON1bits.ASAM = 1;
                while (!IFS0bits.AD1IF){};
                AD1CON1bits.ASAM=0;
                OffData[idx+1]+= ADC1BUF0;
//                t_input-= ADC1BUF0;
                t_input = (OnData[idx] - ((OffData[idx]+OffData[idx+1])>>1))>>2;
//                t_input = t_input>>2;
                data[DATAPREFIXLENGTH+idx] = t_input>=0?t_input:0;
//                idx++;
            }
//            AD1CON1bits.SAMP = 0;

        }

    }
    PORTEbits.RE1=1;
    delay(3);
    PORTEbits.RE1=0;




}

void SetPorts(){

    TRISB = 0xffff;
    initADCL();

    TRISE = 0xf0;

    TRISFbits.TRISF0=0;
    TRISFbits.TRISF1=0;
    TRISFbits.TRISF3=0;
    TRISFbits.TRISF4=0;
    TRISFbits.TRISF5=0;

    PORTFbits.RF3 = 0;
    PORTFbits.RF4 = 1;
}

void delay(unsigned int ticks)
{
    T1CON = 0x8000;
//    T1CON = 0x8020;
    TMR1=0;
    while(TMR1<ticks);
}

void initADCL(){
//    a) Configure port pins as analog inputs and/or select band gap reference inputs (AD1PCFGL<15:0> and AD1PCFGH<1:0>).
    AD1PCFGLbits.PCFG15 = 0;
    AD1PCFGLbits.PCFG14 = 0; //FSR1
    AD1PCFGLbits.PCFG13 = 0; //FSR2
    AD1PCFGLbits.PCFG12 = 0; //FSR3
    AD1PCFGLbits.PCFG11 = 0; //FSR4
    AD1PCFGLbits.PCFG10 = 0; //Spare AN
    AD1PCFGLbits.PCFG9 = 0; //Spare AN
    AD1PCFGLbits.PCFG8 = 0; //Spare AN
    AD1PCFGLbits.PCFG7 = 0; //Spare AN
    AD1PCFGLbits.PCFG6 = 0; //Touch
    //AD1PCFGH

//    b) Select voltage reference source to match expected range on analog inputs (AD1CON2<15:13>).
    AD1CON2bits.VCFG = 0; // VR+ = AVdd  VR- = AVss

//    c) Select the analog conversion clock to match desired data rate with processor clock (AD1CON3<7:0>).
    AD1CON3bits.ADCS = 0; // Tcy = 2*Tosc = 2/24Mhz = 1 / 12Mhz, ADC Conversion Clock TAD = TCY * (ADCS + 1) = 1/12Mhz = 83.3333ns

//    d) Select the appropriate sample/conversion sequence (AD1CON1<7:5> and AD1CON3<12:8>).
    AD1CON1bits.SSRC = 7; //Internal counter ends sampling and starts conversion(auto-convert)
//    AD1CON3bits.SAMC = 1; //Auto-Sample Time = 1 TAD
    AD1CON3bits.SAMC = 23; //Auto-Sample Time = 1 TAD

//    e) Select how conversion results are presented in the buffer (AD1CON1<9:8>).
    AD1CON1bits.FORM = 0; // Data Output Format: Integer

//    f) Select interrupt rate (AD1CON2<5:2>).
    AD1CON2bits.SMPI = 0; //Interruptsatthecompletionofconversionforeachsample/convertsequence


//    g) Turn on A/D module (AD1CON1<15>).
    AD1CHSbits.CH0NA = 0; // CHANNEL 0 NEGATIVE INPUT (MUXA) = VR-
    AD1CHSbits.CH0SA = 15;// CHANNEL 0 POSITIVE INPUTA (MUXA) = AN15
    AD1CHSbits.CH0NB = 0;
//    AD1CSSLbits.CSSL15 = 1;
//    AD1CSSLbits.CSSL14 = 1;
//    AD1CSSLbits.CSSL13 = 1;
//    AD1CSSLbits.CSSL12 = 1;
//    AD1CSSLbits.CSSL11 = 1;
//    AD1CSSLbits.CSSL10 = 1;
//    AD1CSSLbits.CSSL9 = 1;
//    AD1CSSLbits.CSSL8 = 1;
//    AD1CSSLbits.CSSL7 = 1;
//    AD1CSSLbits.CSSL6 = 1;
//    AD1CSSL = 0xffff;
    AD1CSSL = 0;


    AD1CON1bits.ADON = 1;   // Turn on the A/D converter


}

void SendData(){
    if((USBDeviceState < CONFIGURED_STATE)||(USBSuspendControl==1)) return;
    int i;
    for(i=0; i<10; i++)
    {
        while(USBHandleBusy(USBGenericInHandle)){ };
        USBGenericInHandle = USBGenWrite(USBGEN_EP_NUM,(BYTE*)&data[i*32],64);
    }
    while(USBHandleBusy(USBGenericOutHandle));
    {
        USBGenericOutHandle = USBGenRead(USBGEN_EP_NUM,(BYTE*)&OUTPacket[0], USBGEN_EP_SIZE);
        if(OUTPacket[0] != 0)
        {
            SetHaptic(OUTPacket[0]);
            if(OUTPacket[0] == 'w')
            {
                CalibrateTouch();
            }
        }
        if(OUTPacket[0] != 0){

            if(on)
            {
                PORTFbits.RF5=0;
                on = FALSE;
            }
            else
            {
                PORTFbits.RF5=1;
                on = TRUE;
            }
        }
    }

}
/****************************************************************************
  Function:
    void TickInit( void )

  Description:
    This function sets up Timer 4 to generate an interrupt every 10 ms.

  Precondition:
    None

  Parameters:
    None

  Returns:
    None

  Remarks:
    The timer period and prescaler values are defined in HardwareProfile.h,
    since they are dependent on the oscillator speed.
  ***************************************************************************/

void TickInit( void )
{
    //TMR4 = 0;
    PR4 = 2; //TIMER_PERIOD;
   /* T4CON = TIMER_ON | STOP_TIMER_IN_IDLE_MODE | TIMER_SOURCE_INTERNAL |
            GATED_TIME_DISABLED | TIMER_16BIT_MODE | TIMER_PRESCALER_1;*/
	T4CON =  STOP_TIMER_IN_IDLE_MODE | TIMER_SOURCE_INTERNAL |
            GATED_TIME_DISABLED | TIMER_16BIT_MODE | TIMER_PRESCALER_1;

    IFS1bits.T4IF = 0;              //Clear flag
    IEC1bits.T4IE = 1;              //Enable interrupt
  //  T4CONbits.TON = 1;              //Run timer
}




// ******************************************************************************************************
// ************** USB Callback Functions ****************************************************************
// ******************************************************************************************************
// The USB firmware stack will call the callback functions USBCBxxx() in response to certain USB related
// events.  For example, if the host PC is powering down, it will stop sending out Start of Frame (SOF)
// packets to your device.  In response to this, all USB devices are supposed to decrease their power
// consumption from the USB Vbus to <2.5mA each.  The USB module detects this condition (which according
// to the USB specifications is 3+ms of no bus activity/SOF packets) and then calls the USBCBSuspend()
// function.  You should modify these callback functions to take appropriate actions for each of these
// conditions.  For example, in the USBCBSuspend(), you may wish to add code that will decrease power
// consumption from Vbus to <2.5mA (such as by clock switching, turning off LEDs, putting the
// microcontroller to sleep, etc.).  Then, in the USBCBWakeFromSuspend() function, you may then wish to
// add code that undoes the power saving things done in the USBCBSuspend() function.

// The USBCBSendResume() function is special, in that the USB stack will not automatically call this
// function.  This function is meant to be called from the application firmware instead.  See the
// additional comments near the function.

/******************************************************************************
 * Function:        void USBCBSuspend(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Call back that is invoked when a USB suspend is detected
 *
 * Note:            None
 *****************************************************************************/
void USBCBSuspend(void)
{
	//Example power saving code.  Insert appropriate code here for the desired
	//application behavior.  If the microcontroller will be put to sleep, a
	//process similar to that shown below may be used:

	//ConfigureIOPinsForLowPower();
	//SaveStateOfAllInterruptEnableBits();
	//DisableAllInterruptEnableBits();
	//EnableOnlyTheInterruptsWhichWillBeUsedToWakeTheMicro();	//should enable at least USBActivityIF as a wake source
	//Sleep();
	//RestoreStateOfAllPreviouslySavedInterruptEnableBits();	//Preferrably, this should be done in the USBCBWakeFromSuspend() function instead.
	//RestoreIOPinsToNormal();									//Preferrably, this should be done in the USBCBWakeFromSuspend() function instead.

	//IMPORTANT NOTE: Do not clear the USBActivityIF (ACTVIF) bit here.  This bit is
	//cleared inside the usb_device.c file.  Clearing USBActivityIF here will cause
	//things to not work as intended.


    #if defined(__C30__)
    #if 0
        U1EIR = 0xFFFF;
        U1IR = 0xFFFF;
        U1OTGIR = 0xFFFF;
        IFS5bits.USB1IF = 0;
        IEC5bits.USB1IE = 1;
        U1OTGIEbits.ACTVIE = 1;
        U1OTGIRbits.ACTVIF = 1;
        Sleep();
    #endif
    #endif
}


/******************************************************************************
 * Function:        void _USB1Interrupt(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called when the USB interrupt bit is set
 *					In this example the interrupt is only used when the device
 *					goes to sleep when it receives a USB suspend command
 *
 * Note:            None
 *****************************************************************************/
#if 0
void __attribute__ ((interrupt)) _USB1Interrupt(void)
{
    #if !defined(self_powered)
        if(U1OTGIRbits.ACTVIF)
        {
            IEC5bits.USB1IE = 0;
            U1OTGIEbits.ACTVIE = 0;
            IFS5bits.USB1IF = 0;

            //USBClearInterruptFlag(USBActivityIFReg,USBActivityIFBitNum);
            USBClearInterruptFlag(USBIdleIFReg,USBIdleIFBitNum);
            //USBSuspendControl = 0;
        }
    #endif
}
#endif

/******************************************************************************
 * Function:        void USBCBWakeFromSuspend(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The host may put USB peripheral devices in low power
 *					suspend mode (by "sending" 3+ms of idle).  Once in suspend
 *					mode, the host may wake the device back up by sending non-
 *					idle state signalling.
 *
 *					This call back is invoked when a wakeup from USB suspend
 *					is detected.
 *
 * Note:            None
 *****************************************************************************/
void USBCBWakeFromSuspend(void)
{
	// If clock switching or other power savings measures were taken when
	// executing the USBCBSuspend() function, now would be a good time to
	// switch back to normal full power run mode conditions.  The host allows
	// a few milliseconds of wakeup time, after which the device must be
	// fully back to normal, and capable of receiving and processing USB
	// packets.  In order to do this, the USB module must receive proper
	// clocking (IE: 48MHz clock must be available to SIE for full speed USB
	// operation).
}

/********************************************************************
 * Function:        void USBCB_SOF_Handler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USB host sends out a SOF packet to full-speed
 *                  devices every 1 ms. This interrupt may be useful
 *                  for isochronous pipes. End designers should
 *                  implement callback routine as necessary.
 *
 * Note:            None
 *******************************************************************/
void USBCB_SOF_Handler(void)
{
    // No need to clear UIRbits.SOFIF to 0 here.
    // Callback caller is already doing that.
}

/*******************************************************************
 * Function:        void USBCBErrorHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The purpose of this callback is mainly for
 *                  debugging during development. Check UEIR to see
 *                  which error causes the interrupt.
 *
 * Note:            None
 *******************************************************************/
void USBCBErrorHandler(void)
{
    // No need to clear UEIR to 0 here.
    // Callback caller is already doing that.

	// Typically, user firmware does not need to do anything special
	// if a USB error occurs.  For example, if the host sends an OUT
	// packet to your device, but the packet gets corrupted (ex:
	// because of a bad connection, or the user unplugs the
	// USB cable during the transmission) this will typically set
	// one or more USB error interrupt flags.  Nothing specific
	// needs to be done however, since the SIE will automatically
	// send a "NAK" packet to the host.  In response to this, the
	// host will normally retry to send the packet again, and no
	// data loss occurs.  The system will typically recover
	// automatically, without the need for application firmware
	// intervention.

	// Nevertheless, this callback function is provided, such as
	// for debugging purposes.
}


/*******************************************************************
 * Function:        void USBCBCheckOtherReq(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        When SETUP packets arrive from the host, some
 * 					firmware must process the request and respond
 *					appropriately to fulfill the request.  Some of
 *					the SETUP packets will be for standard
 *					USB "chapter 9" (as in, fulfilling chapter 9 of
 *					the official USB specifications) requests, while
 *					others may be specific to the USB device class
 *					that is being implemented.  For example, a HID
 *					class device needs to be able to respond to
 *					"GET REPORT" type of requests.  This
 *					is not a standard USB chapter 9 request, and
 *					therefore not handled by usb_device.c.  Instead
 *					this request should be handled by class specific
 *					firmware, such as that contained in usb_function_hid.c.
 *
 * Note:            None
 *****************************************************************************/
void USBCBCheckOtherReq(void)
{
}//end


/*******************************************************************
 * Function:        void USBCBStdSetDscHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USBCBStdSetDscHandler() callback function is
 *					called when a SETUP, bRequest: SET_DESCRIPTOR request
 *					arrives.  Typically SET_DESCRIPTOR requests are
 *					not used in most applications, and it is
 *					optional to support this type of request.
 *
 * Note:            None
 *****************************************************************************/
void USBCBStdSetDscHandler(void)
{
    // Must claim session ownership if supporting this request
}//end


/******************************************************************************
 * Function:        void USBCBInitEP(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called when the device becomes
 *                  initialized, which occurs after the host sends a
 * 					SET_CONFIGURATION (wValue not = 0) request.  This
 *					callback function should initialize the endpoints
 *					for the device's usage according to the current
 *					configuration.
 *
 * Note:            None
 *****************************************************************************/
void USBCBInitEP(void)
{
    USBEnableEndpoint(USBGEN_EP_NUM,USB_OUT_ENABLED|USB_IN_ENABLED|USB_HANDSHAKE_ENABLED|USB_DISALLOW_SETUP);
    USBGenericOutHandle = USBGenRead(USBGEN_EP_NUM,(BYTE*)&OUTPacket,USBGEN_EP_SIZE);
}

/********************************************************************
 * Function:        void USBCBSendResume(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USB specifications allow some types of USB
 * 					peripheral devices to wake up a host PC (such
 *					as if it is in a low power suspend to RAM state).
 *					This can be a very useful feature in some
 *					USB applications, such as an Infrared remote
 *					control	receiver.  If a user presses the "power"
 *					button on a remote control, it is nice that the
 *					IR receiver can detect this signalling, and then
 *					send a USB "command" to the PC to wake up.
 *
 *					The USBCBSendResume() "callback" function is used
 *					to send this special USB signalling which wakes
 *					up the PC.  This function may be called by
 *					application firmware to wake up the PC.  This
 *					function should only be called when:
 *
 *					1.  The USB driver used on the host PC supports
 *						the remote wakeup capability.
 *					2.  The USB configuration descriptor indicates
 *						the device is remote wakeup capable in the
 *						bmAttributes field.
 *					3.  The USB host PC is currently sleeping,
 *						and has previously sent your device a SET
 *						FEATURE setup packet which "armed" the
 *						remote wakeup capability.
 *
 *					This callback should send a RESUME signal that
 *                  has the period of 1-15ms.
 *
 * Note:            Interrupt vs. Polling
 *                  -Primary clock
 *                  -Secondary clock ***** MAKE NOTES ABOUT THIS *******
 *                   > Can switch to primary first by calling USBCBWakeFromSuspend()

 *                  The modifiable section in this routine should be changed
 *                  to meet the application needs. Current implementation
 *                  temporary blocks other functions from executing for a
 *                  period of 1-13 ms depending on the core frequency.
 *
 *                  According to USB 2.0 specification section 7.1.7.7,
 *                  "The remote wakeup device must hold the resume signaling
 *                  for at lest 1 ms but for no more than 15 ms."
 *                  The idea here is to use a delay counter loop, using a
 *                  common value that would work over a wide range of core
 *                  frequencies.
 *                  That value selected is 1800. See table below:
 *                  ==========================================================
 *                  Core Freq(MHz)      MIP         RESUME Signal Period (ms)
 *                  ==========================================================
 *                      48              12          1.05
 *                       4              1           12.6
 *                  ==========================================================
 *                  * These timing could be incorrect when using code
 *                    optimization or extended instruction mode,
 *                    or when having other interrupts enabled.
 *                    Make sure to verify using the MPLAB SIM's Stopwatch
 *                    and verify the actual signal on an oscilloscope.
 *******************************************************************/
void USBCBSendResume(void)
{
    static WORD delay_count;

    USBResumeControl = 1;                // Start RESUME signaling

    delay_count = 1800U;                // Set RESUME line for 1-13 ms
    do
    {
        delay_count--;
    }while(delay_count);
    USBResumeControl = 0;
}


/*******************************************************************
 * Function:        BOOL USER_USB_CALLBACK_EVENT_HANDLER(
 *                        USB_EVENT event, void *pdata, WORD size)
 *
 * PreCondition:    None
 *
 * Input:           USB_EVENT event - the type of event
 *                  void *pdata - pointer to the event data
 *                  WORD size - size of the event data
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called from the USB stack to
 *                  notify a user application that a USB event
 *                  occured.  This callback is in interrupt context
 *                  when the USB_INTERRUPT option is selected.
 *
 * Note:            None
 *******************************************************************/
BOOL USER_USB_CALLBACK_EVENT_HANDLER(USB_EVENT event, void *pdata, WORD size)
{
    switch(event)
    {
        case EVENT_CONFIGURED:
            USBCBInitEP();
            break;
        case EVENT_SET_DESCRIPTOR:
            USBCBStdSetDscHandler();
            break;
        case EVENT_EP0_REQUEST:
            USBCBCheckOtherReq();
            break;
        case EVENT_SOF:
            USBCB_SOF_Handler();
            break;
        case EVENT_SUSPEND:
            USBCBSuspend();
            break;
        case EVENT_RESUME:
            USBCBWakeFromSuspend();
            break;
        case EVENT_BUS_ERROR:
            USBCBErrorHandler();
            break;
        case EVENT_TRANSFER:
            Nop();
            break;
        default:
            break;
    }
    return TRUE;
}

/****************************************************************************
  Function:
    void GraphReadPotentiometer( void )

  Description:
    This function reads the potentiometer and stores the current value into
    a global variable for later display on the scrolling graph.  If we are
    capturing the data to a flash drive, it also adds a record containing
    the current tick value and the potentiometer reading to the current log
    buffer.

  Precondition:
    None

  Parameters:
    None

  Returns:
    None

  Remarks:
    This routine is called from the tick interrupt.
  ***************************************************************************/

void ReadPotentiometer()
{
	int cnt =0;
	unsigned int i, j, k;

	for(i=0; i<20; i++)
	{
		for(j=0; j<64; j++){

			AD1CON1bits.ADON    = 1;        // Turn on module : Start A/D in continuous mode
			AD1CON1bits.SAMP    = 1;
		    while(!AD1CON1bits.DONE);       // Wait for conversion to complete
		    //potADC              = ADC1BUF0;
			 data[i*64+j] = ADC1BUF0 & 0xff;

		}
	}

}
void ADCInit(void)
{
	TRISBbits.TRISB0 = 1; // analog pin set
	AD1PCFGL = 0x00; // is it right?

	AD1CON1             = 0x00E6;   // Off, Auto sample start, auto-convert
 	AD1CON2             = 0x00;        // AVdd, AVss, int every conversion, MUXA only

 	AD1CON3             = 0x0100;   //1 Tad auto-sample, Tad =2Tcy
    AD1CHS              = 0x0;      // MUXA uses AN0
    AD1CSSL             = 1;        // No scanned inputs

}


//Timer 4 interrupt routine
/****************************************************************************
  Function:
    void __attribute__((interrupt, shadow, auto_psv)) _T4Interrupt(void)

  Description:
    This function updates the tick count and calls ReadCTMU() to monitor the
    touchpads.

  Precondition:
    Timer 4 and the Timer 4 interrupt must be enabled in order for
                    this function to execute.  CTMUInit() must be called before
                Timer 4 and the Timer 4 interrupt are enabled.

  Parameters:
    None

  Returns:
    None

  Remarks:
    None
  ***************************************************************************/
void __attribute__((interrupt, shadow, auto_psv)) _T4Interrupt(void)
{
    // Clear flag
    IFS1bits.T4IF = 0;
    tick= tick<1200? tick+1: 0;
	if(tick == 1200){
		 bufferFlag = 1;
		for(i=0; i<20; i++)
		 {
	            while(USBHandleBusy(USBGenericInHandle)){ };
	            USBGenericInHandle = USBGenWrite(USBGEN_EP_NUM,(BYTE*)&data[i*64],64);
		 }
		 mLED_1_Toggle();
		 mLED_2_Toggle();
	}

	//ReadPotentiometer(tick);
  //  ReadCTMU(); get the sensor values
}
