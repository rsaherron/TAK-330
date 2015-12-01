/*
 * main_drive.c
 *
 *  Created on: Nov 13, 2015
 *     Authors: Taylor Brown,  Adam Herron, and Kent Williams
 * Description: Code to be loaded on the main drive PIC24 microcontroller to
 * control the navigation of the Supple Shooter
 * 
 * --- Base PIC24 Resources ---
 * Pin 1: Master/Clear
 * Pin 2:
 * Pin 3:
 * Pin 4: OC2 --> Step PWM to Pololu ESC's --> Left and Right Tracks
 * Pin 5: OC3 --> PWM (currently unused)
 * Pin 6: _LATB2 (Digital Output) --> Dir on Left Track Pololu ESC Board
 * Pin 7: _LATA2 (Digital Output) --> Dir on Right Track Pololu ESC Board
 * Pin 8: _LATA3 (Digital Output) --> Trig on Front Range Finder
 * Pin 9: _LATB4 (Digital Output) --> Trig on Lateral Range Finder
 * Pin 10: 
 * Pin 11:
 * Pin 12:
 * Pin 13:
 * Pin 14: OC1 --> PWM (unused)
 * Pin 15: RB12 (Digital Input) <-- Echo from front RangeFinder
 * Pin 16: RB13 (Digital Input) <-- Echo from lateral RangeFinder
 * Pin 17: AN10 RB14 (Analog Input) <-- Input from Left Base IR Sensor
 * Pin 18: AN9 RB15 (Analog Input) <-- Input from Right Base IR Sensor
 * Pin 19: VSS
 * Pin 20: VDD
 * Timer 1: OC1 PWM
 * Timer 2: OC2 PWM
 * Timer 3: OC3 PWM
 * Timer 4: Range Finders Period/ Game Timer
 * Timer 5: Range Finder Triggers/Echos
 */

#include <p24F16KA301.h>
#include <stdlib.h>

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
void _ISR _T4Interrupt(void);
void _ISR _T5Interrupt(void);
int RF_state = 0;
int RF_trigger_state = 0;   // used in RF Interrupt Service Routine
int game_time = 0;          // Elapsed Game Time (in seconds)


int main()
{
    int state = 1;          // Main State of the robot
	int destination = 1;    // sub-state for traveling state
    int L_dir = 1;          // Left track direction (1=forward, 0=backward)
    int R_dir = 0;          // Right track direction (1=forward, 0=backward)
    int track_speed = 40;    // PWM period of STEP pulses to track pololu boards
    int max_speed = 40;     // constant for minimum value of track_speed
    int IR_LOW = 1500;      // min (ambient) analog IR reading
    int IR_DIFF = 15;       // minimum difference between Left_IR and Right_IR
    int last_L_IR = ADC1BUF10;  // pervious reading of new_L_IR
    int last_R_IR = ADC1BUF9;   // pervious reading of new_R_IR
    int new_L_IR = ADC1BUF10;   // new reading from left analog IR sensor on base
    int new_R_IR = ADC1BUF9;    // new reading from right analog IR sensor on base
    float L_IR = 0;         // average of new and last IR readings
    float R_IR = 0;         // average of new and last IR readings
    int last_F_echo_state = _RB12;      // last value of Front RF echo input
    int last_L_echo_state = _RB13;      // last value of Lateral  RF echo input
    int new_F_echo_state = _RB12;   // read state of RB12
    int new_L_echo_state = _RB13;   // read state of RB13
    int F_count = 0;        // count on TMR5
    int L_count = 0;        // count on TMR5
    float count2range = 3.342e-4;   // convert 2 microsecond counts to meters
    int last_F_range = 0;   // pervious reading of new_F_range
    int last_L_range = 0;   // pervious reading of new_L_range
    int new_F_range = 0;    // new reading from front analog range finder
    int new_L_range = 0;    // new reading from lateral analog range finder
    float F_range = 1000;      // average of new and last IR readings
    float L_range = 1000;      // average of new and last IR readings
    float range_DIFF = 0.003;   // uncertainty in rangefinder
    float center2corner = 0.6*0.707;
    int center_travel_state = 0;    // Sub-state for Center destination of Traveling State
    int ball_count = 0;     // number of balls currently in hopper
    float wrap_up_time = 100;   // Time at which the robot stops collecting/shooting and heads for the center
    int collecting_state = 0;   // Sub state for the Ball Collecting routine
    float beam_DIST = 0.0761;   // Distance from corner to IR Collector Trigger Beam
    float F2L_DIST = 0.17;      // Distance from F_IR sensor to L_IR sensor axis (in meters) (ADJUST ME!!!)
    float L2F_DIST = 0.05;      // Distance from L_IR sensor to F_IF sensor axis (in meters) (ADJUST ME!!!)
    
    while(1)
    {
        if(game_time >= wrap_up_time)
        {
            state = 0;
        }
        switch(state)
        {
            // State 1: Initializing
            case 1:
                config_IO();
                config_ad();
                config_PWM();
                destination = 1;// Center
                L_dir = 1;      // Forward
                R_dir = 1;      // Forward
                track_speed = 3125;

                state = 7;
                
                break;
                
            //State 2: Traveling    
            case 2:
                switch(destination)
                {
                    // Destination 1: Center from Random Location
                    case 1:
                        switch(center_travel_state)
                        {
                            // State 0: orienting perpendicular to nearest walls
                            case 0:
                                if(F_range > 0.6 || L_range > 0.6)
                                {
                                    R_dir = 0;
                                    L_dir = 1;
                                    track_speed = max_speed;
                                }
                                else if(F_range <= 0.6 && L_range <= 0.6)
                                {
                                    track_speed = max_speed * 2;
                                    if(abs(last_F_range - F_range) < range_DIFF)
                                    {
                                        center_travel_state = 1;
                                        track_speed = 0;
                                    }
                                    else if (last_F_range < F_range)
                                    {
                                        L_dir = ~L_dir;
                                        R_dir = ~R_dir;
                                    }
                                }
                                break;
                                
                            // State 1: drive forward or backward until adjusted F_RF == adjusted L_RF
                            case 1:
                                if(abs(F_range - L_range) < range_DIFF) // adjust these for center of rotation!!!
                                {
                                    center_travel_state = 2;
                                    track_speed = max_speed*2;
                                    L_dir = 1;
                                    R_dir = 0;
                                }
                                else if(F_range > L_range)      // adjust these for center of rotation!!!
                                {
                                    L_dir = 1;
                                    R_dir = 1;
                                    track_speed = max_speed*2;
                                }
                                else
                                {
                                    L_dir = 0;
                                    R_dir = 0;
                                    track_speed = max_speed*2;
                                }
                                break;
                                
                            // State 2: Turn 45 degrees to face corner
                            case 2:
                                if(abs(last_F_range - F_range) < range_DIFF)
                                {
                                    center_travel_state = 3;
                                    track_speed = max_speed * 2;
                                    R_dir = 0;
                                    L_dir = 0;
                                }
                                else if (last_F_range < F_range)
                                {
                                    R_dir = ~R_dir;
                                    L_dir = ~L_dir;
                                }
                                break;
                            // State 3: Back up to center of arena
                            case 3:
                                if (abs(center2corner - F_range) < range_DIFF)
                                {
                                    destination = 2;    // Arrived at center of arena
                                    if(game_time >= wrap_up_time)   // If in the last 5 seconds, stall here forever
                                    {
                                        track_speed = 0;
                                        L_dir = 1;
                                        R_dir = 1;
                                    }
                                    else
                                    {
                                        state = 3;          // Else, Switch Overall State to 'Locating Ball Dispenser'
                                        track_speed = 0;
                                        L_dir = 1;
                                        R_dir = 1;
                                    }
                                }
                                else if (F_range > center2corner)
                                {
                                    track_speed = max_speed;    // Drive forward
                                    L_dir = 1;
                                    R_dir = 1;
                                }
                                else
                                {
                                    track_speed = max_speed;    // Drive Backward
                                    L_dir = 0;
                                    R_dir = 0;
                                }
                                break;
                                
                        }
                        break;
                        
                    // Destination 2: Ball Dispenser from Center  
                    case 2:
                        // travel to ball dispenser
                        // when you get there, change the state
                        break;
                }
                break;
            
            // State 3: Locating Ball Dispenser
            case 3:
                new_L_IR = ADC1BUF10;                   // Pin 17: AN10 (Analog Input) <-- Input from Left Base IR Sensor
                new_R_IR = ADC1BUF9;                    // Pin 18: AN9 (Analog Input) <-- Input from Right Base IR Sensor
                L_IR = (last_L_IR + new_L_IR)/2.0;      // Slight Digital Average Filtering
                R_IR = (last_R_IR + new_R_IR)/2.0;      // Slight Digital Average Filtering
                last_L_IR = new_L_IR;                   // Set the new_IR readings to the last_IR readings for the next loop
                last_R_IR = new_R_IR;
                
                if (L_IR < IR_LOW && R_IR < IR_LOW)     // If LIR = RIR = 0 --> Rotate Clockwise Fast
                {
                    track_speed = max_speed;
                    L_dir = 1;
                    R_dir = 0;
                }
                else if (R_IR > IR_LOW && R_IR - L_IR > IR_DIFF)    // If R_IR > L_IR --> Rotate CW Slower
                {
                    track_speed = max_speed * 2;
                    L_dir = 1;
                    R_dir = 0;
                }
                else if (L_IR > IR_LOW && L_IR - R_IR > IR_DIFF)    // If L_IR > R_IR --> Rotate CCW Slower
                {
                    track_speed = max_speed * 2;
                    L_dir = 0;
                    R_dir = 1;
                }
                else if (L_IR > IR_LOW && R_IR > IR_LOW && abs(L_IR - R_IR) < IR_DIFF ) // stop rotating when L_IR ~= R_IR > 0
                {
                    track_speed = 0;
                    L_dir = 1;
                    R_dir = 1;
                    state = 4;      // Start Collecting Balls
                }
                
                break;
            
            // State 4: Collecting Balls    
            case 4:
                if (ball_count >= 6)
                {
                    state = 5;      // stop collecting balls  once you get to 6
                    break;
                }
                switch(collecting_state)
                {
                    // case 0: Driving from the center of the arena to trigger the ball collector
                    case 0:
                        // If the tank is triggering the beam, stop and go to the next case
                        // Else, keep driving forward
                        break;
                    // case 1: Back up a bit and get ready to trigger the beam again
                    case 1:
                        // If the tank is far enough back that it's not triggering the beam, stop and go back to case 0
                        // Else, keep driving backward
                        break;
                }
                
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
            
            // test drive state
            case 7:
                L_dir = 1;      // Forward
                track_speed = F_range;
                R_dir = 1;      // Forward
                break;
                
        }
        
        // Handle Range Finders and Game Timer
        /* Pin 8: _LATA3 (Digital Output) --> Trig on Front Range Finder
         * Pin 9: _LATB4 (Digital Output) --> Trig on Lateral Range Finder
         * Pin 15: RB12 (Digital Input) <-- Echo from front RangeFinder
         * Pin 16: RB13 (Digital Input) <-- Echo from lateral RangeFinder
         * Timer 4 (TMR4) has a tick of 2 microseconds and period of 0.1 second
         * Timer 5 (TMR5) does all kinds of crazy things
         * RF_trigger_state determines what Timer 5 is doing
         * Trig pulses should be 10 microseconds long (5 ticks)
         * Distance = V*T/2; where V is the speed of sound (343.2m/s) and T is the Echo pulse length
         */ 
        
        // Handle Front RF
        if(RF_trigger_state == 2)
        {
            new_F_echo_state = _RB12;
            if(new_F_echo_state != last_F_echo_state)
            {
                if(last_F_echo_state == 0)
                    F_count = TMR5;
                else
                {
                    last_F_range = F_range;
                    new_F_range = (TMR5 - F_count)*count2range + F2L_DIST;
                    F_range = (last_F_range + new_F_range)/2.0;     // digital signal averaging
                }
            }
            last_F_echo_state = new_F_echo_state;
        }
        
        // Handle Lateral RF
        if(RF_trigger_state == 5)
        {
            new_L_echo_state = _RB13;
            if(new_L_echo_state != last_L_echo_state)
            {
                if(last_L_echo_state == 0)
                    L_count = TMR5;
                else
                {
                    last_L_range = L_range;
                    new_L_range = (TMR5 - L_count)*count2range + L2F_DIST;
                    L_range = (L_range + new_L_range)/2.0;     // digital signal averaging
                }
            }
            last_L_echo_state = new_L_echo_state;
        }
        
        // Set speed and direction of motors
//        PR1 = ADC1BUF10;
//        OC1RS = PR1;
//        OC1R = PR1/2.0;
        PR2 = track_speed;
        OC2RS = PR2;
        OC2R = PR2/2.0;
//        PR3 = track_speed;
//        OC3RS = PR3;
//        OC3R = PR3/2.0;
        if(L_dir == 1)
            _LATB2 = 1;
        else
            _LATB2 = 0;
        if(R_dir == 1)
            _LATA2 = 0;
        else
            _LATA2 = 1;
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
    AD1CSSL = 0;        // AD1CSSL<15:0> -- clear lower channels to scan
    AD1CSSH = 0;        // AD1CSSH<15:0> -- clear upper channels to scan
    _CSS9 = 1;          // A/D scan AN9
    _CSS10 = 1;         // AN10
    _CSS11 = 1;         // AN11
    _CSS12 = 1;         // AN12
    
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
    // Configure Timer 3
    T3CONbits.TON = 1;          //enable Timer3
    T3CONbits.TCS = 0;          //Set source to internal clock
    T3CONbits.TCKPS = 0b11;     //Select prescale value of 256:1 - Tick Period of 64 microseconds
    PR3 = 2;                    //Set initial timer period to 128 microseconds
    TMR3 = 0;                   //Set timer count to 0
    // Configure Timer 4
    T4CONbits.TON = 1;
    T4CONbits.TCS = 0;          //Set source to internal clock
    T4CONbits.TCKPS = 0b01;     //Select prescale value of 8:1 - Tick Period of 2 microseconds
    PR4 = 50000;                //Set timer period to 0.1 seconds (enough for both range finder to fire and receive their signal)
    TMR4 = 0;                   //Set timer count to 0
    // Configure Timer4 interrupt 
    _T4IP = 4;                  // Select interrupt priority 
    _T4IE = 1;                  // Enable interrupt 
    _T4IF = 0;                  // Clear interrupt flag 
    // Configure Timer 5
    T5CONbits.TON = 1;
    T5CONbits.TCS = 0;          //Set source to internal clock
    T5CONbits.TCKPS = 0b01;     //Select prescale value of 8:1 - Tick Period of 2 microseconds
    PR5 = 50000;                //Set initial timer period to 0.1 seconds
    TMR5 = 0;                   //Set timer count to 0
    // Configure Timer5 interrupt 
    _T5IP = 5;                  // Select interrupt priority 
    _T5IE = 1;                  // Enable interrupt 
    _T5IF = 0;                  // Clear interrupt flag 


  // PWM Period = [Value + 1] x TCY x (Prescaler Value) //

  // Configure Output Compare 1
    OC1CON1bits.OCTSEL = 0b100;     //Select Timer1 to be timer source
    OC1CON1bits.OCM = 0b110;        //Select Edge-Aligned PWM mode
    OC1CON2bits.SYNCSEL = 0b01011;  //Select Timer1 as synchronization source
    OC1RS = PR1/2;                  //Set duty cycle to 1/2 period
  // Configure Output Compare 2
    OC2CON1bits.OCTSEL = 0b000;     //Select Timer2 to be timer source
    OC2CON1bits.OCM = 0b110;        //Select Edge-Aligned PWM mode
    OC2CON2bits.SYNCSEL = 0b01100;  //Select Timer2 as synchronization source
    OC2RS = PR2/2;                  //Set duty cycle to 1/2 period
  // Configure Output Compare 3
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
    
    //Set Analog in pin to input mode
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

void __attribute__((interrupt, no_auto_psv)) _T4Interrupt(void)
{
    _T4IF = 0;      // Clear interrupt flag
    game_time += 0.1;
    switch(RF_state)
    {
        // State 0: Reading Front RF
        case 0:
            RF_trigger_state = 3;
            RF_state = 1;
            PR5 = 5;
            _T5IF = 1;
            break;
            
        // State 1: Reading Lateral RF
        case 1:
            RF_trigger_state = 0;
            RF_state = 0;
            PR5 = 5;
            _T5IF = 1;
            break;
    }
}

void __attribute__((interrupt, no_auto_psv)) _T5Interrupt(void) 
{ 
    _T5IF = 0;      // Clear interrupt flag 
    switch(RF_trigger_state)
    {
        // State 0: time to raise Front RF_trigger
        case 0:
            PR5 = 5;        // 10 microsecond pulse
            _LATA3 = 1;
            RF_trigger_state = 1;
            TMR5 = 0;
            break;

        // State 1: time to drop Front RF_trigger and listen for Front RF echo
        case 1:
            PR5 = 50000;
            _LATA3 = 0;
            RF_trigger_state = 2;
            TMR5 = 0;
            break;

        // State 2: listening for Front RF echo
        case 2:
            // This isn't supposed to ever interrupt here. If it does, just go to state 3
            RF_trigger_state = 3;
            break;
        // State 3: time to raise Lateral RF_trigger   
        case 3:
            PR5 = 5;        // 10 microsecond pulse
            _LATB4 = 1;
            RF_trigger_state = 4;
            TMR5 = 0;
            break;

        // State 4: time to drop Lateral RF_trigger and listen for Lateral RF echo
        case 4:
            PR5 = 50000;
            _LATB4 = 0;
            RF_trigger_state = 5;
            TMR5 = 0;
            break;
        // state 5: listening for lateral RF echo
        case 5:
            // This isn't ever supposed to interrupt here. If it does, just go to state 0
            RF_trigger_state = 0;
            break;
    }
} 