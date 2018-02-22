/***************************************************
Name:- Mark Kearney & John McShane
Date last modified:- Feb 2017
Filename:- PI_Controller.c
Program Description:- Using a PI Controller to control a geared DC Motor
 
 * PORTA: POT to ADC
 * PORTB: Buttons
 * PORTC: LEDs, Capture, PWM
 * PORTD: LCD
 
 
*****************************************************************************************************/

#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include "../../Debounce_library/Buttons_Debounce.h"
#include "../../LCD_library/lcdlib_2016.h"
#include <plib/timers.h>
#define _XTAL_FREQ 8000000

/************************************************
			Global Variables
*************************************************/
const unsigned char msg_ary[10][16] = {"Dsired Val>", "Actual Val>", 
                                        "POT","Enter to Accept"};
bit TICK_E = 0; //flag for TICK event

const unsigned char * problem = "Problem";
const unsigned char * startup = "Ready to go";
										  

/************************************************
			Function Prototypes
*************************************************/
void Initial(void);
void Window(unsigned char num);
void delay_s(unsigned char secs);



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
        WriteTimer0(40960); 
        INTCONbits.TMR0IF = 0;  // clear this interrupt condition
        
        //TICK_E
        count_sample++;
        if(count_sample == 10){ //If overflow 10 times (10*10ms=100ms, TICK event)
            TICK_E = 1;
            count_sample=0;
        }
        
        //Heartbeat signal
        count_test++;
        if(count_test == 100){
            PORTCbits.RC7 = ~PORTCbits.RC7;   //check the timer overflow rate
            count_test = 0;                   //Toggle every 1 second (heartbeat))
        }
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
        
    struct{
        unsigned char Desired;
        unsigned char Actual;
    } motor;
    
    motor.Desired = 50;
    motor.Actual = 25;
    
    typedef  enum {RUN = 0,UPDATE_DESIRED} state_t; //two states
    state_t  state = RUN;
    
    Initial();
    lcd_start ();
    lcd_cursor ( 0, 0 ) ;
    lcd_print ( startup ) ;
    
    delay_s(2);
    
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
                    state = UPDATE_DESIRED; //state transition
                    Window(1);             //OnEntry action
                }
                
				break;
			case UPDATE_DESIRED: 
				if (MENU_E){
                    state = RUN;  //state transition
                    Window(0);              //OnEntry action
                }
				break;
			default: 
				if (MENU_E){
                    state = RUN;
                    Window(0);
                }
				break;
		}
		
		
		switch(state)	
		{
			case RUN: 
				lcd_cursor ( 12, 0 ) ;    //state actions
                lcd_display_value(motor.Desired);
                lcd_cursor ( 12, 1 ) ;
                lcd_display_value(motor.Actual);
				
				break;
			case UPDATE_DESIRED: 
                if (ENTER_E)          //state actions with guard
                    motor.Desired = POT_Val;
				lcd_cursor ( 10, 0 ) ;
                lcd_display_value(motor.Desired);				
                
				break;			
			default: 
				lcd_cursor ( 0, 0 ) ;
                lcd_clear();
				lcd_print ( problem );
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
    
	LATC = 0xff;
	delay_s(1);
	LATC = 0x00;
    
    //0n, 16bit, internal clock(Fosc/4)
    OpenTimer0( TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_1);
    WriteTimer0(45536);  //65,536 - 24,576  //overflows every 10mS
    ei();
    
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

