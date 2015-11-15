/*
 * main_drive.c
 *
 *  Created on: Nov 13, 2015
 *     Authors: Taylor Brown,  Adam Herron, and Kent Williams
 * Description: Code to be loaded on the main drive PIC24 microcontroller to
 * control the navigation of the Supple Shooter
 * 
 * Base PIC24 PinOut Inventory:
 * Pin 1: Master/Clear
 * Pin 2:
 * Pin 3:
 * Pin 4: OC2 --> Step PWM to Pololu ESC --> Left Track
 * Pin 5: OC3 --> Step PWM to Pololu ESC --> Right Track
 * Pin 6: _LATB2 (Digital Output) --> Dir on Left Track Pololu ESC Board
 * Pin 7: _LATA2 (Digital Output) --> Dir on Right Track Pololu ESC Board
 * Pin 8:  
 * Pin 9: 
 * Pin 10: 
 * Pin 11:
 * Pin 12:
 * Pin 13:
 * Pin 14: OC1 --> PWM (unused)
 * Pin 15: AN12 RB12 (Analog Input) <-- Input from front RangeFinder
 * Pin 16: AN11 RB13 (Analog Input) <-- Input from lateral RangeFinder
 * Pin 17: AN10 RB14 (Analog Input) <-- Input from Left Base IR Sensor
 * Pin 18: AN9 RB15 (Analog Input) <-- Input from Right Base IR Sensor
 * Pin 19: VSS
 * Pin 20: VDD
 * 
 */

#include <p24F16KA301.h>

_FOSCSEL(FNOSC_FRC & SOSCSRC_DIG);        // Select 8 MHz Oscillator and enable pins 9 & 10 for IO
_FOSC(OSCIOFNC_OFF);        //enable pin 8 for IO
_FICD(ICS_PGx2);

//------------------------------------------------------------------------
// A/D Configuration Function
//
// This function configures the A/D to read from two channels in auto
// conversion mode.
//------------------------------------------------------------------------
void config_ad(void);
void config_PWM(void);
void config_IO(void);

int main()
{
    int state = 1;
	int destination = 1;
    int L_dir = 1;
    int L_speed = 0;
    int R_dir = 1;
    int R_speed = 0;
    int max_speed = 20;
    int IR_threshold = 10;
    int last_L_IR = 0;
    int last_R_IR = 0;
    int new_L_IR = ADC1BUF10;
    int new_R_IR = ADC1BUF9;
    int L_IR = 0;
    int R_IR = 0;
    
    while(1)
	{
        switch(state)
        {
            // State 1: Initializing
            case 1:
                config_IO();
                config_ad();
                config_PWM();
                destination = 1;// Center
                L_dir = 1;      // Forward
                L_speed = 3125;
                R_dir = 1;      // Forward
                R_speed = 3125;
                
                state = 3;      // Go to State 2
                
                break;
                
            //State 2: Traveling    
            case 2:     
                switch(destination)
                {
                    // Destination 1: Center
                    case 1:
                        // travel to center
                        // when you get there, change the state
                        break;
                        
                    // Destination 2: Ball Dispenser    
                    case 2:
                        // travel to ball dispenser
                        // when you get there, change the state
                        break;
                }
                break;
            
            // State 3: Locating Ball Dispenser
            case 3:
                /* Pin 17: AN10 (Analog Input) <-- Input from Left Base IR Sensor
                 * Pin 18: AN9 (Analog Input) <-- Input from Right Base IR Sensor */
                L_IR = (last_L_IR + new_L_IR)/2.0;
                R_IR = (last_R_IR + new_R_IR)/2.0;
                
                if (L_IR < IR_threshold && R_IR < IR_threshold) // If LIR = RIR = 0 --> Rotate Clockwise Fast
                {
                    L_speed = max_speed;
                    L_dir = 1;      // Forward
                    R_speed = max_speed;
                    R_dir = 0;      // Reverse
                }
                else if (R_IR > IR_threshold && R_IR - L_IR > IR_threshold) // If R_IR > L_IR --> Rotate Clockwise Slower
                {
                    L_speed = max_speed / 2;
                    L_dir = 1;      // Forward
                    R_speed = max_speed / 2;
                    R_dir = 0;      // Reverse
                }
                else if (L_IR > IR_threshold && L_IR - R_IR > IR_threshold)
                {
                    L_speed = max_speed / 2;
                    L_dir = 0;      // Reverse
                    R_speed = max_speed / 2;
                    R_dir = 1;      // Forward
                }
                else if (L_IR > IR_threshold && R_IR > IR_threshold && abs(L_IR - R_IR) < IR_threshold ) // stop rotating when L_IR = R_IR > 0
                {
                    L_speed = 0;
                    L_dir = 1;      // Forward
                    R_speed = 0;
                    R_dir = 1;      // Forward
                }
                
                last_L_IR = new_L_IR; // Set the new_IR readings to the last_IR readings for the next loop
                last_R_IR = new_R_IR;
                break;
            
            // State 4: Collecting Balls    
            case 4:
                // do the things
                break;
            
            // State 5: Locating Goal    
            case 5:
                // do the things
                break;
            
            // State 6: Shooting Balls    
            case 6:
                // do the things
                break;
                
            break;        
        }
        
        // Set speed and direction of motors
        PR1 = ADC1BUF10;
        OC1RS = PR1;
        OC1R = PR1/2.0;
        PR2 = L_speed;
        OC2RS = PR2;
        OC2R = PR2/2.0;
        PR3 = R_speed;
        OC3RS = PR3;
        OC3R = PR3/2.0;
        
        if(L_dir == 1)
            _LATB2 = 1;
        else
            _LATB2 = 0;
        if(R_dir == 1)
            _LATA2 = 1;
        else
            _LATA2 = 0;
        
	}

return 0;
}

void config_ad(void)
{

    // AD1CHS register
    _CH0NA = 0;         // AD1CHS<7:5> -- Use VDD as negative input

    // AD1CON1 register
    _ADON = 1;          // AD1CON1<15> -- Turn on A/D
    _ADSIDL = 0;        // AD1CON1<13> -- A/D continues while in idle mode?
    _MODE12 = 1;        // AD1CON1<10> -- 12-bit or 10-bit?
    _FORM = 0;          // AD1CON1<9:8> -- Output format
    _SSRC = 7;          // AD1CON1<7:4> -- Auto conversion (internal counter)
    _ASAM = 1;          // AD1CON1<2> -- Auto sampling

    // AD1CSSL registers
    AD1CSSL = 0;        // AD1CSSL<15:0> -- Select lower channels to scan
    AD1CSSH = 0;        // AD1CSSH<15:0> -- Select upper channels to scan
    _CSS1 = 1;
    _CSS15 = 1;


    // AD1CON2 register
    _PVCFG = 0;         // AD1CON2<15:14> -- Use VDD as positive ref voltage
    _NVCFG = 0;         // AD1CON2<13> -- Use VSS as negative ref voltage
    _BUFREGEN = 1;      // AD1CON2<11> -- Results stored using channel indexed      
                        // mode -- AN1 result is stored in ADC1BUF1, AN2 result
                        // is stored in ADC1BUF2, etc.
    _CSCNA = 1;         // AD1CON2<10> -- Scans inputs specified in AD1CSSx
                        // registers instead of using channels specified
                        // by CH0SA bits in AD1CHS register
    _ALTS = 0;          // AD1CON2<0> -- Sample MUXA only
    _SMPI = 2;          // AD1CON2<6:2> -- Interrupts at the conversion for
                        // every other sample (number of channels - 1)

    // AD1CON3 register
    _ADRC = 0;          // AD1CON3<15> -- Use system clock
    _SAMC = 1;          // AD1CON3<12:8> -- Auto sample every A/D period TAD
    _ADCS = 0x3F;          // AD1CON3<7:0> -- A/D period TAD = 64*TCY
}

void config_PWM(void)
{
     // Configure Timer 1
    _TON = 1;                   //enable Timer1
    _TCS = 0;                   //Set source to internal clock
    _TCKPS = 0b11;              //Select prescale value of 256:1 - Tick Period of 64 microseconds
    PR1 = 2;                    //Set initial timer period to 128 microseconds
    TMR1 = 0;                   //Set timer count to 0
    // Configure Timer 2
    T2CONbits.TON = 1;          //enable Timer2
    T2CONbits.TCS = 0;          //Set source to internal clock
    T2CONbits.TCKPS = 0b11;     //Select prescale value of 256:1 - Tick Period of 64 microseconds
    PR2 = 2;                    //Set initial timer period to 128 microseconds
    TMR2 = 0;                   //Set timer count to 0
    // Configure Time 3
    T3CONbits.TON = 1;          //enable Timer2
    T3CONbits.TCS = 0;          //Set source to internal clock
    T3CONbits.TCKPS = 0b11;     //Select prescale value of 256:1 - Tick Period of 64 microseconds
    PR3 = 2;                    //Set initial timer period to 128 microseconds
    TMR3 = 0;                   //Set timer count to 0

  // PWM Period = [Value + 1] x TCY x (Prescaler Value) //

  // Configure Output Compare 1
    OC1CON1bits.OCTSEL = 0b100;     //Select Timer1 to be timer source
    OC1CON1bits.OCM = 0b110;        //Select Edge-Aligned PWM mode
    OC1CON2bits.SYNCSEL = 0b01011;  //Select current OCx as synchronization source
    OC1RS = PR1/2;                  //Set duty cycle to 1/2 period
  // Configure Output Compare 2 (Left track)
    OC2CON1bits.OCTSEL = 0b000;     //Select Timer2 to be timer source
    OC2CON1bits.OCM = 0b110;        //Select Edge-Aligned PWM mode
    OC2CON2bits.SYNCSEL = 0b01100;  //Select Timer2 as synchronization source
    OC2RS = PR2/2;                  //Set duty cycle to 1/2 period
  // Configure Output Compare 3 (Left track)
    OC3CON1bits.OCTSEL = 0b001;     //Select Timer3 to be timer source
    OC3CON1bits.OCM = 0b110;        //Select Edge-Aligned PWM mode
    OC3CON2bits.SYNCSEL = 0b01101;  //Select Timer3 as synchronization source
    OC3RS = PR3/2;                  //Set duty cycle to 1/2 period
}

void config_IO(void)
{
 /* Pin 15: AN12 RB12 (Analog Input) <-- Input from front RangeFinder
 * Pin 16: AN11 RB13 (Analog Input) <-- Input from lateral RangeFinder
 * Pin 17: AN10 RB14 (Analog Input) <-- Input from Left Base IR Sensor
 * Pin 18: AN9 RB15 (Analog Input) <-- Input from Right Base IR Sensor*/
    // Configure the digital I/O ports
    TRISA = 0;          //Set A ports to output
    TRISB = 0;          //Set B ports to output
    ANSA = 0;           //disables Port A analog input
    ANSB = 0;           //disables Port B analog input
    
    //Set Analog in puts to input
    _TRISB12 = 1;
    _TRISB13 = 1;
    _TRISB14 = 1;
    _TRISB15 = 1;
    
    //Enable Analog in read from analog in pins
    _ANSB12 = 1;
    _ANSB13 = 1;
    _ANSB14 = 1;
    _ANSB15 = 1;
}