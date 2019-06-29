/*==================================CPEG222=============================
* Program:		Project4.c
* Authors: 		Yuchen Tang & Mingrui Liu
* Date: 		3/1/2017
* Inputs : On board Btn1 & Btn2
* Outputs: One on board LED (LD1 to 4) and Pmod 8LD on Port B of the MX7 board
========================================================================*/ 
#include <plib.h>                   //include peripheral library    
#include <math.h>                   //include math library
#include "dsplib_dsp.h" //include digital signal processing library
#include "fftc.h" //include FFT library
#pragma config ICESEL = ICS_PGx1	//configure on board licensed debugger
#pragma config FNOSC = PRIPLL		//configure system clock 80 MHz
#pragma config POSCMOD = EC, FPLLIDIV = DIV_2,FPLLMUL = MUL_20,FPLLODIV = DIV_1
#pragma config FPBDIV = DIV_1		//configure peripheral bus clock to 80 MHz
#define SYS_FREQ  (80000000L)       // 80MHz system clock
#define CORE_TICK_RATE	    200000//(SYS_FREQ/2/REFRESH_SSD)
#define SAMPLE_FREQ 18450
     
/* ------------------------------------------------------------ */
/*				Definitions                                     */
/* ------------------------------------------------------------ */        
// SSD Pmod1 (2 rightmost SSDs)using the bottom rows of JA & JB jumpers
#define SegA1       LATBbits.LATB7
#define SegB1       LATBbits.LATB8
#define SegC1       LATBbits.LATB9
#define SegD1       LATBbits.LATB10
#define SegE1       LATEbits.LATE4
#define SegF1       LATEbits.LATE5
#define SegG1       LATEbits.LATE6
#define DispSel1    LATEbits.LATE7 //Select between the cathodes of the 2 SSDs
// SSD Pmod2 (2 leftmost SSDs)using the bottom rows of JC & JD jumpers
#define SegA2       LATBbits.LATB15
#define SegB2       LATDbits.LATD5
#define SegC2       LATDbits.LATD4
#define SegD2       LATBbits.LATB14
#define SegE2       LATDbits.LATD1
#define SegF2       LATDbits.LATD2
#define SegG2       LATDbits.LATD3
#define DispSel2    LATDbits.LATD12 //Select between the cathodes of the 2 SSDs
//Pmod 8LD connected to Port JF
#define LD1         LATFbits.LATF12
#define LD2         LATFbits.LATF5
#define LD3         LATFbits.LATF4
#define LD4         LATFbits.LATF13
#define LD5         LATEbits.LATE9   
#define LD6         LATAbits.LATA1
#define LD7         LATAbits.LATA4
#define LD8         LATAbits.LATA5
#define Led1        LATGbits.LATG12 //Define Led1 to appropriate port bit
#define Btn1        PORTGbits.RG6   //Define Btn1 to appropriate port bit
#define N 2048//the number of samples stored (needs to be 2^n for FFT)
#define fftc fft16c2048//FFT for 1024 samples on 16 bit data

unsigned char display_char[]={  //used for display digit function
	0b0111111,	//0
	0b0000110,	//1
	0b1011011,	//2
	0b1001111,	//3
	0b1100110,	//4
	0b1101101,	//5
	0b1111101,	//6
	0b0000111,	//7
	0b1111111,	//8
	0b1101111,	//9
    0b0000000	//clear, off
};
/* ------------------------------------------------------------ */
/*				Forward Declarations                    		*/
/* ------------------------------------------------------------ */
void delay_ms(int ms);
void DisplayChar(char value,int SSD);
void DisplaySigLevel(int volume);
int computeFFT(int16c *sampleBuffer);
void mips_fft16(int16c *dout, int16c *din, int16c *fftc,int16c *scratch, int log2N);
int readADC(int ch);
int readADC2();

int left_SSD = 0;     //track which SSD to light (1=left/0=right)
int ones = 0;         //Variable to track character for first SSD
int tens = 0;         //Variable to track character for 2nd SSD
int hunds = 0;         //Variable to track character for 3rd SSD
int thous = 0;         //Variable to track character for 4th SSD
int Val = 0;
int Peak = 750;      //set the high end of the AnalogInput level (<=1023)
int Offset = 220;    //set the low end of the AnalogInput level (>=0)
int LEDs = 0;           // the number of LEDs to light on the 8LD Pmod
int log2N = 11; // i.e. 2^11 = N = 2048
int mute = 1;      
int tcount = 0;
int maxSignal = 0;
int freq;
int i;
int buttonlock = 0;
int sampleIndex;
int16c sampleBuffer[N];
int16c scratch[N]; //intermediate array
int16c din[N];
int16c dout[N];// intermediate array holds computed FFT until transmission
int singleSidedFFT[N];//intermediate array holds computed single side FFT
short freqVector[N];//array that stores the corresponding frequency of each point in
//frequency domain

char SSD1 = 0b0000000;        //SSD setting for 1st SSD (LSD)
char SSD2 = 0b0000000;        //SSD setting for 2nd SSD
char SSD3 = 0b0000000;        //SSD setting for 3rd SSD 
char SSD4 = 0b0000000;        //SSD setting for 4th SSD (MSD)

/* ------------------------------------------------------------ 
**	__ISR for the CoreTimer
**	Parameters:
**		none
**	Return Value:
**		none
**	Description:
**		Display 2 characters on the 4 SSDs and then reset the timer
** ------------------------------------------------------------ */
void __ISR(_CORE_TIMER_VECTOR, IPL2SOFT) CoreTimerHandler(void)
{   
    if(!left_SSD){
        DisplayChar(SSD2,1);
        DisplayChar(SSD4,3);
    }
    else{
        DisplayChar(SSD1,0);
        DisplayChar(SSD3,2);
    }
    left_SSD = ! left_SSD;
       mCTClearIntFlag();  // clear the interrupt flag
       UpdateCoreTimer(CORE_TICK_RATE);
    
}

void __ISR(_TIMER_3_VECTOR, ipl1) _T3Interrupt(void) {
 //The sample buffer receives the readADC() as the real part
    if (!mute){
        sampleBuffer[tcount].re = readADC2();
        sampleBuffer[tcount].im = 0;
        if (tcount ==(N-1)){
            tcount = 0;
        }
        else{
            tcount++;
        }
    }
    IFS0CLR = 0b1000000000000; // clear interrupt flag
}

int main() {
    DDPCONbits.JTAGEN = 0; //Shutoff JTAG
    SYSTEMConfig(SYS_FREQ, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE); //  Configure device for max performance
    OpenCoreTimer(CORE_TICK_RATE);// set up the core timer interrupt with a priority of 2 and zero sub-priority
    mConfigIntCoreTimer((CT_INT_ON | CT_INT_PRIOR_2 | CT_INT_SUB_PRIOR_0));  
    INTEnableSystemMultiVectoredInt();  // enable multi-vector interrupts
        
    PORTSetPinsDigitalOut (IOPORT_G, BIT_12|BIT_13| BIT_14|BIT_15); //Set LD1 through LD4 as digital output
    PORTSetPinsDigitalIn (IOPORT_G, BIT_6 |BIT_7); //Set Btn1 and Btn2 as inputs
    PORTSetPinsDigitalOut (IOPORT_B,BIT_7|BIT_8| BIT_9| BIT_10| BIT_14| BIT_15);// Set MX7 Port A as output for SSD1R
   	PORTSetPinsDigitalOut (IOPORT_E,BIT_4|BIT_5| BIT_6|BIT_7);// Set MX7 Port B as output for SSD1R
   	PORTSetPinsDigitalOut (IOPORT_D,BIT_1|BIT_2| BIT_3|BIT_4 |BIT_5| BIT_12);// Set MX7 Port C and D as output for SSD2   
    PORTSetPinsDigitalOut (IOPORT_F,BIT_12|BIT_4|BIT_5|BIT_13); //Set MX7 Port F as output for 8LD Pmod
    PORTSetPinsDigitalOut (IOPORT_E,BIT_9);//Set MX7 Port F as output for 8LD Pmod
    PORTSetPinsDigitalOut (IOPORT_A,BIT_1|BIT_4|BIT_5);//Set MX7 Port F as output for 8LD Pmod
    
    
    T3CON = 0x0070; // Set  timer 1 prescale to 1:256,
    TMR3 = 0x0; // Clear timer register
    PR3 = 0x10; // Load period register
    mT3SetIntPriority(1);
    mT3ClearIntFlag();  // clear the interrupt flag 
    INTEnableSystemMultiVectoredInt();  // enable multi-vector interrupts
    mT3IntEnable(1);
    T3CONSET = 0x8000;

    AD1PCFGbits.PCFG3 = 0;    // AN3 is an ADC10 pin
    AD1CON1bits.ADON = 1;     // turn on A/D converter
    AD1CON3bits.ADCS = 2;     // ADC clock period is Tad = 2*(ADCS+1)*Tpb
                              //  Tad = 2*3*12.5ns = 75ns

    while (1) {    
        Led1 = !mute; //Led1 off
        
        if (Btn1 && !buttonlock){  //when press button1, start collect  
            for (i=0; i<N/2; i++){
                freqVector[i] = i*(SAMPLE_FREQ/2)/((N/2) - 1);
            }
            delay_ms(20);
            mute = (mute + 1) % 2;  //When press button1, Led1 on.
            buttonlock = 1;
        }
        if(tcount== N && Btn1 && !buttonlock){
            buttonlock=1;
            maxSignal=0;
            for(sampleIndex=0;sampleIndex<N;sampleIndex++){
                if(sampleBuffer[sampleIndex].re>maxSignal){
                    maxSignal = sampleBuffer[sampleIndex].re;
                }
            }
            LEDs = floor(((maxSignal-Offset)*8.0)/Peak);
            DisplaySigLevel(LEDs);
            if(LEDs>0){
                Val = freqVector[computeFFT(&sampleBuffer)];    
            }
            else{
            Val=0;
            }
            tcount=0;
        }
        else if (!Btn1 && buttonlock){
            delay_ms(20);
            buttonlock = 0; 
        }
        
        Val = readADC(3);    // sample and convert pin 3
        LEDs = floor((( Val - Offset) * 8.0) / (Peak - Offset) + 0.5);
        DisplaySigLevel(LEDs);
        freq = freqVector[computeFFT(&sampleBuffer)];

        thous = floor(freq/1000); //calculate the thousands character
        if (thous > 0)  {     //display the thousands character if not 0
            SSD4 = display_char[thous];
            delay_ms(300);}
        else
            SSD4 = 0b0000000; //if 0 blank the SSD
        hunds = floor((freq%1000)/100);//calculate the hundreds character
       
        if (thous==0 && hunds==0)
            SSD3 = 0b0000000; //blank the SSD if thou and hund = 0
        else
            SSD3 = display_char[hunds];
            
        tens = floor(freq%100/10); //calculate the tens character
        if (thous==0 && hunds==0 && tens==0)
            SSD2 = 0b0000000; //blank the SSD if thou,hund, and tens = 0
        else
             SSD2 = display_char[tens];
            
        ones = floor(freq%10); //calculate the tens character
        if (thous==0 && hunds==0 && tens==0 && ones==0)
            SSD1 = 0b0000000;//blank the SSD if all values are 0
        else
            SSD1 = display_char[ones];
            }   
    
  return 0;
}

/* ------------------------------------------------------------ 
**	readADC
**	Parameters:
**		ch			- AN channel to read (0 - 15)
**	Return Value:
**		int         - digital value from 10 bit ADC (0-1023)
**	Description:
**		reads the voltage at pin ch and returns the ADC value
** ------------------------------------------------------------ */
int readADC( int ch){
    if(!mute){
        AD1CHSbits.CH0SA = ch;         // 1. select analog input
        AD1CON1bits.SAMP = 1;          // 2. start sampling
        T1CON = 0x8000;  TMR1 = 0;     // 3. wait for sampling time
        while (TMR1 < 100);            //
        AD1CON1bits.SAMP = 0;          // 4. start the conversion
        while (!AD1CON1bits.DONE);     // 5. wait conversion complete
        return ADC1BUF0;   
    }// 6. read result
 }

int readADC2(){
 AD1CON1bits.SAMP = 1;          // 2. start sampling
 T1CON = 0x8000;  TMR1 = 0;     // 3. wait for sampling time
 while (TMR1 < 100);            //
 AD1CON1bits.SAMP = 0;          // 4. start the conversion
 while (!AD1CON1bits.DONE);     // 5. wait conversion complete
 return ADC1BUF0;               // 6. read result
 }

int computeFFT(int16c *sampleBuffer){
 int i;
 int dominant_freq=1;

 mips_fft16(dout, sampleBuffer, fftc, scratch, log2N);
//computer N point FFT
 for(i = 0; i < N/2; i++){ //compute single sided fft
    singleSidedFFT[i] = 2 * ((dout[i].re*dout[i].re) + (dout[i].im*dout[i].im));
 }
 for(i = 1; i < N/2; i++){
//find the index of dominant frequency
    if (singleSidedFFT[dominant_freq]<singleSidedFFT[i])
    dominant_freq=i;
    }
 return dominant_freq;
}
/* ------------------------------------------------------------ 
**	DisplayChar
**	Parameters:
**		num			- character (0 thru F) to be displayed
**      SSD         - SSD to put the number on, 3 to 0 left to right
**	Return Value:
**		none
**	Description:
**		Display the numbers 0 to 9 by lighting the proper segments
** ------------------------------------------------------------ */
void DisplayChar(char value,int SSD)
{
    if(SSD > 1){      
        DispSel2 = SSD-2;
        SegA2 = value & 1;
        SegB2 = (value >> 1) & 1;
        SegC2 = (value >> 2) & 1;
        SegD2 = (value >> 3) & 1;
        SegE2 = (value >> 4) & 1;
        SegF2 = (value >> 5) & 1;
        SegG2 = (value >> 6) & 1;
    }
    else{
        DispSel1 = SSD;
        SegA1 = value & 1;
        SegB1 = (value >> 1) & 1;
        SegC1 = (value >> 2) & 1;
        SegD1 = (value >> 3) & 1;
        SegE1 = (value >> 4) & 1;
        SegF1 = (value >> 5) & 1;
        SegG1 = (value >> 6) & 1;
    }
}

/* ------------------------------------------------------------ 
**	delay_ms
**	Parameters:
**		ms			- amount of milliseconds to delay (based on 80 MHz SSCLK)
**	Return Value:
**		none
**	Description:
**		Create a delay by counting up to counter variable
** ------------------------------------------------------------ */
void delay_ms(int ms)
{
	int		i,counter;
	for (counter=0; counter<ms; counter++){
        for(i=0;i<1250;i++){}   //software delay 1 millisec
    }
}

/* ------------------------------------------------------------ 
**	DisplaySigLevel
**	Parameters:
**		volume			- signal level (0 thru 8)
**	Return Value:
**		none 
**	Description:
**		Light the 8 LEDs on Pmod relative to signal level
** ------------------------------------------------------------ */
void DisplaySigLevel(int volume){
    switch(volume){        //light the appropriate number of LEDs
        case 0:LD1=LD2=LD3=LD4=LD5=LD6=LD7=LD8=0;break;
        case 1:LD1=1;LD2=LD3=LD4=LD5=LD6=LD7=LD8=0;break;
        case 2:LD1=LD2=1;LD3=LD4=LD5=LD6=LD7=LD8=0;break;
        case 3:LD1=LD2=LD3=1;LD4=LD5=LD6=LD7=LD8=0;break;
        case 4:LD1=LD2=LD3=LD4=1;LD5=LD6=LD7=LD8=0;break;
        case 5:LD1=LD2=LD3=LD4=LD5=1;LD6=LD7=LD8=0;break;
        case 6:LD1=LD2=LD3=LD4=LD5=LD6=1;LD7=LD8=0;break;
        case 7:LD1=LD2=LD3=LD4=LD5=LD6=LD7=1;LD8=0;break;
        case 8:LD1=LD2=LD3=LD4=LD5=LD6=LD7=LD8=1;break;
        default:break;
        }
}

