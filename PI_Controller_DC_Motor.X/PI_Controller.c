/***************************************************
Name:- Mark Kearney & John McShane
Date last modified:- 22 March 2018
Filename:- PI_Controller.c
Program Description:- Using a PI Controller to control a geared DC Motor
 
 * PORTA: POT to ADC
 * PORTB: Buttons
 * PORTC: LEDs, Capture, PWM
 * PORTD: LCD
 * This program will control a DC Motors speed.
 
 
*****************************************************************************************************/

#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include "../../Debounce_library/Buttons_Debounce.h"
#include "../../LCD_library/lcdlib_2016.h"
#include <plib/timers.h>
#include <plib/adc.h>
#define _XTAL_FREQ 8000000

/************************************************
			Global Variables
*************************************************/
const unsigned char msg_ary[10][16] = {"Dsired Val>", "Actual Val>",    //window msgs
                                        "POT"       ,"Enter to Accept",
                                        "UPDATED!"  ,""};
bit TICK_E = 0; //flag for TICK event

const unsigned char * problem = "Problem";  //error msg
const unsigned char * startup = "Ready to go"; //starup msg
unsigned int captured = 0; //16 bit value to allow data to be read from the CCPR1H:CCPR1L on interrupt										  

typedef struct{         //structure for controller
    float Kp; //proportional gain
    float Ki; //integral gain
    float Kp_Ki; //Kp + Ki
    signed char en; //error
    signed char en_1; //previous error
    unsigned char un; //output
    unsigned char un_1; //previous output
}Controller;
Controller control1; //create instance of controller
 struct{           //structure for motor
        unsigned char Desired;
        unsigned char Actual;
    } motor;
    
    
/************************************************
			Function Prototypes
*************************************************/
void Initial(void);
void Window(unsigned char num);
void delay_s(unsigned char secs);
void setup_PWM(void);
void setup_capture(void);
void controller_func(void);

/************************************************
 Interrupt Function 
*************************************************/
unsigned char count_test =0;
unsigned char count_sample = 0;
void __interrupt myIsr(void)
{ 
    //Timer overflows every 10mS
    // only process timer-triggered interrupts
    if(INTCONbits.TMR0IE && INTCONbits.TMR0IF) {
        
        Find_Button_Press();       //check the buttons every 10mS
        WriteTimer0(40960); //preload
        INTCONbits.TMR0IF = 0;  // clear this interrupt condition
        
        //TICK_E
        count_sample++;
        if(count_sample == 10){ //If overflow 10 times (10*10ms=100ms, TICK event every 100ms)
            TICK_E = 1;
            count_sample=0;
            
            controller_func();  //call control function
            CCPR2L = control1.un; //update PWM duty cycle 
        }
        
        //Heartbeat signal
        count_test++;
        if(count_test == 100){
            PORTAbits.RA4 = ~PORTAbits.RA4;   //check the timer overflow rate
            count_test = 0;                   //Toggle every 1 second (heartbeat))       
        }
             
    }
    
     if(PIR1bits.CCP1IF & PIE1bits.CCP1IE) //capture interrupt
    {
        PIR1bits.CCP1IF = 0; //reset flag
        TMR1H  = 0x00;  //reset timer
        TMR1L  = 0x00;
        
        captured = CCPR1L + CCPR1H*256;  //read captured value     
    }   
    
}


//declare Button
Bit_Mask Button_Press;	


/************************************************
			Macros
*************************************************/
#define MENU_E Button_Press.B0 //switch menu
#define ENTER_E Button_Press.B1 //enter button
/*****************************************
 			Main Function
******************************************/

void main ( void ) 
{
    unsigned char POT_Val = 0;
    unsigned int rev_s;
    unsigned short long tclk = 250000; // F_ins = 2000000, prescale by 8 = 250000
    
   
    
    motor.Desired = 40; //default value
    motor.Actual = 0;
    
    typedef  enum {RUN = 0,UPDATE_DESIRED} state_t; //two states
    state_t  state = RUN;
    
    Initial();
    lcd_start ();
    lcd_cursor ( 0, 0 ) ; 
    lcd_print ( startup ) ; //startup msg
    setup_capture();
    setup_PWM();
    delay_s(2);
    
    //initial values for controller
    control1.Kp = 0.4;
    control1.Ki = 0.3;
    control1.Kp_Ki = control1.Kp +control1. Ki;
    control1.en = 0;
    control1.en_1 = 0;
    control1.un = 50;
    control1.un_1 = 50;
    CCPR2L = control1.un;

    
    //Initial LCD Display
    Window(0);
    lcd_cursor ( 12, 0 ) ;
    lcd_display_value(motor.Desired);
    lcd_cursor ( 12, 1 ) ;
    lcd_display_value(motor.Actual);
    
    while(1)
    {
		
		//wait for a button Event or tick event
		while(!Got_Button_E && !TICK_E);
        
		switch(state)	
		{
			case RUN: 
				if (MENU_E){
                    state = UPDATE_DESIRED; //state transition to update state
                    Window(1);             //OnEntry action
                    ADCON0bits.ADON = 1; //turn on adc
                }
                
				break;
			case UPDATE_DESIRED: 
				if (MENU_E){
                    state = RUN;  //state transition to RUN state
                    Window(0);              //OnEntry action
                    ADCON0bits.ADON = 0; //turn off adc
                }
				break;
			default: 
				if (MENU_E){
                    state = RUN; //default state
                    Window(0);
                }
				break;
		}
		
		
		switch(state)	//state actions
		{
			case RUN: 
                captured == 0 ? rev_s = 0 : rev_s = tclk/captured; //calc speed
                captured = 0; //reset captured value
                motor.Actual = rev_s; //update actual value
                
				lcd_cursor ( 12, 0 ) ;    
                lcd_display_value(motor.Desired);
                
                
                lcd_cursor ( 12, 1 ) ;
                lcd_display_value(motor.Actual);
                 
                 
                 
				break;
			case UPDATE_DESIRED: 
                POT_Val = ADRESH >> 2;// read 6 MSb of adc value
                if(POT_Val > 50)
                    POT_Val = 50; //cap speed at 50
                if (ENTER_E){          
                    motor.Desired = POT_Val; //update desired value
                    Window(2);
                    lcd_cursor ( 0, 1 ) ;
                     lcd_display_value(motor.Desired);
                    delay_s(1);
                    Window(1);
                }
				lcd_cursor ( 10, 0 ) ;
                lcd_display_value(POT_Val);				
                SelChanConvADC(ADC_CH0);    //start new conversion (every tick event)
				break;			
			default: 
				lcd_cursor ( 0, 0 ) ;
                lcd_clear();
				lcd_print ( problem ); //error msg
				break;
		}
		
        Button_Press.Full = 0;  //Clear all events since only allowing one button event at a time
                                //which will be dealt with immediately
        TICK_E = 0; //clear tick event
    }
}   


void Initial(void){
    
    
    
    ADCON1 = 0x0F; // digital pins
    
	TRISB = 0xFF; //Buttons
	TRISC = 0x00;   //LEDS
    
    OSCCONbits.SCS1 = 1;    //Internal oscillator block
    OSCCONbits.IRCF2 = 1;   //Set to 8MHz..
    OSCCONbits.IRCF1 = 1;
    OSCCONbits.IRCF0 = 1;
    
    //0n, 16bit, internal clock(Fosc/4)
    OpenTimer0( TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_1);
    WriteTimer0(45536);  //65,536 - 24,576  //overflows every 10mS
    ei();
    
    //adc
    OpenADC(ADC_FOSC_RC & ADC_LEFT_JUST & ADC_4_TAD,
            ADC_CH0 & ADC_INT_OFF & ADC_REF_VDD_VSS,
            ADC_1ANA);
    TRISAbits.RA4 = 0; //heartbeat led is output
}

// controller function 
void controller_func(void)
{
    control1.en_1 = control1.en; //save previous error
    control1.en = motor.Desired - motor.Actual;   //calc new error
    control1.un_1 = control1.un ;   //save previous output
    
    control1.un = (control1.un_1) + (control1.Kp_Ki * control1.en) - (control1.Kp * control1.en_1); //calc new output value
    if(control1.un > 100)  
        control1.un = 100; //cap at 100% duty cycle
}


//display line 1 & line 2 msg from array
void Window(unsigned char num)
{
    lcd_clear();
    lcd_cursor ( 0, 0 ) ;	
	lcd_print ( msg_ary[num*2]);// select line 1 msg
    lcd_cursor ( 0, 1 ) ;
    lcd_print ( msg_ary[(num*2)+1]);// select line 2 msg
}


void delay_s(unsigned char secs)
{
    unsigned char i,j;
    for(j=0;j<secs;j++)
    {
        for (i=0;i<25;i++)
                __delay_ms(40);  //max value is 40 since this depends on the _delay() function which has a max number of cycles
        
    }
}

void setup_PWM(void)
{
    CCP2CON = 0b00001100; //pwm mode
    PR2 = 100;  // frequency will be 20kHz
    T2CON = 0b00000100;  //tmr2 is turned on but not prescaled
    TRISCbits.RC1 = 0; //ccp2 pin output  
    

}


void setup_capture(void)
{
    //PORT C
    TRISC = 0x04; //pin 2 as input, rest as output
    
    //TIMER 1 
    T1CONbits.RD16 = 1;   //16bit clock
    T1CONbits.T1CKPS = 0b11; //prescale by 8
    T1CONbits.TMR1ON = 1;   //turn on tmr 1
    
    //CAPTURE
    CCP1CONbits.CCP1M = 0b0101; //capture on rising edge
    PIE1bits.CCP1IE = 1; //enable capture interrupt
    
    
    //INTERRUPTS
    INTCONbits.GIEL     = 1;    //Peripheral Interrupt Enable (PEIE)
    INTCONbits.GIE      = 1;    //Enable interrupts
}