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
 * Pin 6: RB2 (Digital Output) --> M0 on both Pololu ESC boards (Optional)
 * Pin 7: RA2 (Digital Output) --> M1 on both Pololu ESC boards (Optional)
 * Pin 8: AN13 (Analog Input) <-- Input from front RangeFinder 
 * Pin 9: AN14 (Analog Input) <-- Input from lateral RangeFinder
 * Pin 10: 
 * Pin 11:
 * Pin 12:
 * Pin 13:
 * Pin 14: OC1 --> 
 * Pin 15:
 * Pin 16:
 * Pin 17: AN10 (Analog Input) <-- Input from Left Base IR Sensor
 * Pin 18: AN9 (Analog Input) <-- Input from Right Base IR Sensor
 * Pin 19: VSS
 * Pin 20: VDD
 * 
 */

#include <p24F16KA301.h>

_FOSCSEL(FNOSC_FRC);        // Select 15 MHz Oscillator

//------------------------------------------------------------------------
// A/D Configuration Function
//
// This function configures the A/D to read from two channels in auto
// conversion mode.
//------------------------------------------------------------------------
void config_ad(void);

int main()
{
    unsigned char state = 'initializing';
	unsigned char destination = 'center';
    unsigned char L_dir = 'forward';
    int L_speed = 0;
    unsigned char R_dir = 'for';
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
            case 'initializing':
                config_ad();
                state = 'traveling';
                destination = 'center';
                L_dir = 'forward';
                L_speed = 0;
                R_dir = 'forward';
                R_speed = 0;
                
                break;
                
            case 'traveling':
                switch(destination)
                {
                    case 'center':
                        // travel to center
                        // when you get there, change the state
                        break;
                    case 'ball dispenser':
                        // travel to ball dispenser
                        // when you get there, change the state
                        break;
                }
                break;
                
            case 'locating dispenser':
                /* Pin 17: AN10 (Analog Input) <-- Input from Left Base IR Sensor
                 * Pin 18: AN9 (Analog Input) <-- Input from Right Base IR Sensor */
                L_IR = (last_L_IR + new_L_IR)/2.0;
                R_IR = (last_R_IR + new_R_IR)/2.0;
                
                if (L_IR < IR_threshold && R_IR < IR_threshold) // If LIR = RIR = 0 --> Rotate Clockwise Fast
                {
                    L_speed = max_speed;
                    L_dir = 'forward';
                    R_speed = max_speed;
                    R_dir = 'reverse';
                }
                else if (R_IR > IR_threshold && R_IR - L_IR > IR_threshold) // If R_IR > L_IR --> Rotate Clockwise Slower
                {
                    L_speed = max_speed / 2;
                    L_dir = 'forward';
                    R_speed = max_speed / 2;
                    R_dir = 'reverse';
                }
                last_L_IR = new_L_IR;
                last_R_IR = new_R_IR;
                break;
                
            case 'collecting balls':
                // do the things
                break;
                
            case 'locating goal':
                // do the things
                break;
                        
            case 'shooting balls':
                // do the things
                break;
                
                      
        }
//	if(_RA0 == 0)				//If Pin 2 is low then set PWM to 1 ms duty cycle
//	{
//	OC2R = 0x1F4;
//	}
//
//	if(_RA0 == 1)				//If Pin 2 is high then set PWM to 2ms duty cycle
//	{
//	OC2R = 0x3E8;
//	}

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