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
 * Pin 5: OC3 --> PWM Signal to Cannon Feed Servo Motor
 * Pin 6: _LATB2 (Digital Output) --> Dir on Left Track Pololu ESC Board
 * Pin 7: _LATA2 (Digital Output) --> Dir on Right Track Pololu ESC Board
 * Pin 8:
 * Pin 9: AN15 RB4 (Analog Input) --> Input From Turret IR Sensor
 * Pin 10:
 * Pin 11: _LATB7 (Digital Output) --> Trig on Front Range Finder
 * Pin 12: _LATB8 (Digital Output) --> Trig on Lateral Range Finder
 * Pin 13: _LATB9 (Digital Output) --> MOSFET Trigger to cannon motors
 * Pin 14: OC1 --> PWM signal to Turret Servo Motor
 * Pin 15: RB12 (Digital Input) <-- Echo from front RangeFinder
 * Pin 16: RB13 (Digital Input) <-- Echo from lateral RangeFinder
 * Pin 17: AN10 RB14 (Analog Input) <-- Input from Left Base IR Sensor
 * Pin 18: AN9 RB15 (Analog Input) <-- Input from Right Base IR Sensor
 * Pin 19: GND -- VSS
 * Pin 20: 3.3 V -- VDD
 * 
 * Timer 1: OC1 PWM
 * Timer 2: OC2 PWM
 * Timer 3: OC3 PWM
 * Timer 4: Range Finders Period/ Game Timer
 * Timer 5: Range Finder Triggers/Echos
 * 
 * Ribbon Cable from Turret to Base:
 * Balck --> GND -- VSS
 * White --> IR_T
 * 
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
void __attribute__ ((__interrupt__)) _IC3Interrupt(void);
void config_IC(void);
int RF_state = 0;
int RF_trigger_state = 0;   // used in RF Interrupt Service Routine
float game_time = 0.0;        // Elapsed Game Time (in seconds)
int IC_count = 0;
float F_range = 0.0;
int range_count = 0;
float count2range = 3.342e-4;   // convert 2 microsecond counts to meters

// Adjustable Parameters
int max_speed = 40;         // constant for minimum value of track_speed
int turret_min = 200;      // Turret minimum angle (absolute min is 500)
int turret_mid = 700;      // Turret midpoint angle in timer counts (NEEDS ADJUSTMENT!!!)
int turret_max = 1275;      // Turret maximum angle (absolute is 1000)
float collector_pause = 0.5;// Wait time between ball collection cycles (NEEDS ADJUSTMENT!!!!)
int collector_angle = 500;  // Turret Servo motor displacement during ball collection (NEEDS ADJUSTMENT!!!)
int cannon_min = 700;      // Cannon Servo minimum angle (Absolute Min is 250))
int cannon_mid = 900;      // Cannon Servo Turret midpoint angle in timer counts (NEEDS ADJUSTMENT!!!)
int cannon_max = 1100;      // Cannon Servo maximum angle (absolute max is 1200)
float cannon_pause = 0.5;   // Wait time between cannon servo position changes (Adjust Me)
float beam_DIST = 0.0761;   // Distance from corner to IR Collector Trigger Beam
float F2L_DIST = 0.17;      // Distance from F_IR sensor to L_IR sensor axis (in meters) (ADJUST ME!!!)
float L2F_DIST = 0.05;      // Distance from L_IR sensor to F_IF sensor axis (in meters) (ADJUST ME!!!)
float max_tracking_step = 15; // Maximum Angular step (in timer counts) which the turret takes while tracking (ADJUST ME!!!)
float tracking_delay = 0.1;
float IR_adjustment = 0;    // ADJUST ME!!! How much greater if R_IR than L_IR?
int T_IR_min = 1400;        // Minimum value of Turret IR sensor to fire a ball (ADJUST ME!!!)


int main()
{
    int state = 1;          // Main State of the robot
	int destination = 1;    // sub-state for traveling state
    int L_dir = 1;          // Left track direction (1=forward, 0=backward)
    int R_dir = 0;          // Right track direction (1=forward, 0=backward)
    int track_speed = 40;    // PWM period of STEP pulses to track pololu boards
    int IR_LOW = 1500;      // min (ambient) analog IR reading
    int IR_DIFF = 15;       // minimum difference between Left_IR and Right_IR
    int last_L_IR = ADC1BUF10;  // pervious reading of new_L_IR
    int last_R_IR = ADC1BUF9;   // pervious reading of new_R_IR
    int last_T_IR = ADC1BUF15;  // previous reading from turret analog IR sensor
    int new_L_IR = ADC1BUF10;   // new reading from left analog IR sensor on base
    int new_R_IR = ADC1BUF9;    // new reading from right analog IR sensor on base
    int new_T_IR = ADC1BUF15;   // new reading from turret analog IR sensor
    float L_IR = 0;         // average of new and last left IR readings
    float R_IR = 0;         // average of new and last right IR readings
    float T_IR = 0;         // average of new and last turret IR readings
    int last_F_echo_state = _RB12;      // last value of Front RF echo input
    int last_L_echo_state = _RB13;      // last value of Lateral  RF echo input
    int new_F_echo_state = _RB12;   // read state of RB12
    int new_L_echo_state = _RB13;   // read state of RB13
    int F_count = 0;        // count on TMR5
    int L_count = 0;        // count on TMR5
    int last_F_range = 0;   // pervious reading of new_F_range
    int last_L_range = 0;   // pervious reading of new_L_range
    int new_F_range = 0;    // new reading from front analog range finder
    int new_L_range = 0;    // new reading from lateral analog range finder
    float L_range = 1000;      // average of new and last IR readings
    float range_DIFF = 0.003;   // uncertainty in rangefinder
    float center2corner = 0.6*0.707;    // distance from the center to the corner of the arena
    int center_travel_state = 0;    // Sub-state for Center destination of Traveling State
    int ball_count = 0;     // number of balls currently in hopper
    float end_of_round = 105;   // Time (on game timer at which the round ends
    int collecting_state = 0;   // Sub state for the Ball Collecting routine
    float last_game_time = 0;   // beginning of ball collector timer
    float turret_angle = turret_max-100;  // angle (in timer counts) of the turret servo motor
    int cannon_angle = cannon_mid;  // angle (in timer counts) of the cannon loading servo motor
    int turret_tracking = 0;        // Is the turret currently seeking the goal (1-Yes/0-N0)?
    int tracking_step = max_tracking_step;  // Angular step which the turret will take during a loop
    int cannon_motors = 0;    // Are the shooting motors spinning (1-Yes/0-No)
    int shooting_state = 0;     // Sub state of the shooting balls state
    int switch_count = 0;       // number of times since the turret went active that the turret has switched directions
    float last_cannon_time = 0;     // Last game_time that the turret fired
    float last_tracking_time = 0;
    
    while(game_time < end_of_round)
    {
        switch(state)
        {
            // test drive state
            case 0:
                L_dir = 1;      // Forward
                track_speed = 0;
                R_dir = 1;      // Forward
                turret_tracking = 1;
                turret_angle = turret_min + 169.3*L_range;
                cannon_angle = cannon_max;
                cannon_motors = 0;
                break;
                
            // State 1: Initializing
            case 1:
                config_IO();
                config_ad();
                config_IC();
                config_PWM();
                destination = 1;// Center
                L_dir = 1;      // Forward
                R_dir = 1;      // Forward
                track_speed = 3125;

                state = 0;      // GO TO TESTING STATE
                
                break;
                
            //State 2: Traveling to Center
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
                                    if(game_time >= end_of_round - 5)   // If in the last 5 seconds, stall here forever
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
                if(game_time + 5 >= end_of_round)   // Abort Ball collection and drive to center if in the last 5 seconds of the game
                {
                    state = 1;
                    destination = 1;
                    break;
                }
                
                new_L_IR = ADC1BUF10;       // Pin 17: AN10 (Analog Input) <-- Input from Left Base IR Sensor
                new_R_IR = ADC1BUF9;        // Pin 18: AN9 (Analog Input) <-- Input from Right Base IR Sensor
                L_IR = (last_L_IR + new_L_IR)/2.0;      // Slight Digital Average Filtering
                R_IR = (last_R_IR + new_R_IR)/2.0 - IR_adjustment;  // Adjust for differences in IR sensors
                last_L_IR = new_L_IR;       // Set the new_IR readings to the last_IR readings for the next loop
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
                    state = 5;      // stop collecting balls once you get to 6
                    shooting_state = 0;
                    turret_tracking = 1;
                    break;
                }
                switch(collecting_state)
                {
                    // case 0: Driving from the center of the arena to trigger the ball collector
                    case 0:
                        turret_angle = turret_mid;
                        turret_tracking = 0;
                        if(F_range <= 0.25*center2corner) // (NEEDS ADJUSTMENT!!!)
                        {
                            collecting_state = 1;
                            break;
                        }
                        else
                        {
                            track_speed = max_speed;
                            L_dir = 1;
                            R_dir = 1;
                        }
                        break;
                    // case 1: pivot the turret to get another ball
                    case 1:// turn the turret some amount
                        // start counting on the game timer to give the turret a chance to reach the destination
                        // if the timer is reached, turn the turret back to the midpoint angle
                        if(game_time > last_game_time + 2*collector_pause)
                        {
                            turret_angle = turret_mid;
                            last_game_time = game_time;
                            // Turn the turret back to the midpoint and reset the timer
                        }
                        else if(game_time > last_game_time + collector_pause)
                        {
                            turret_angle = turret_mid + collector_angle;
                        // turn the turret some amount
                        }
                        break;
                }
                
                break;
            
            // State 5: Shooting Balls    
            case 5:
                turret_tracking = 1;
                cannon_motors = 1;
                
                if(ball_count <= 0)     // Stop if out of balls!
                {
                    state = 3;
                    shooting_state = 0;
                    turret_tracking = 0;
                    cannon_angle = cannon_mid;
                    break;
                }

                switch(shooting_state)
                {
                    // Shooting State 0: Initialize and drive to center
                    case 0:
                        switch_count = 0;
                        if(abs(F_range - center2corner) < range_DIFF)
                        {
                            track_speed = 0;
                            L_dir = 1;
                            R_dir = 1;
                            shooting_state = 1;
                        }
                        else if (F_range > center2corner)
                        {
                            track_speed = max_speed*2;
                            L_dir = 1;
                            R_dir = 1;
                        }
                        else
                        {
                           track_speed = max_speed;
                           L_dir = 0;
                           R_dir = 0; 
                        }
                        
                        break;
                    // Shooting State 1: Finding the active goal
                    case 1:
                        if(T_IR > T_IR_min && switch_count > 10)    // Wait for turret to find active goal 
                        {
                            shooting_state = 2;
                            last_cannon_time = game_time;
                            cannon_angle = cannon_min;
                            ball_count -= 1;
                        }
                        break;
                    // Shooting State 2: Shooting Balls
                    case 2:
                        if(game_time >= last_cannon_time + cannon_pause*2)
                        {
                            cannon_angle = cannon_min;
                            ball_count -= 1;
                            last_cannon_time = game_time;
                        }
                        else if (game_time >= last_cannon_time + cannon_pause)
                        {
                            cannon_angle = cannon_max;
                        }
                        break;
                }
                break;                
        }
        
        // Handle Range Finders and Game Timer
        /* Pin 11: _LATB7 (Digital Output) --> Trig on Front Range Finder
         * Pin 12: _LATB8 (Digital Output) --> Trig on Lateral Range Finder
         * Pin 15: RB12 (Digital Input) <-- Echo from front RangeFinder
         * Pin 16: RB13 (Digital Input) <-- Echo from lateral RangeFinder
         * Timer 4 (TMR4) has a tick of 2 microseconds and period of 0.1 second
         * Timer 5 (TMR5) does all kinds of crazy things
         * RF_trigger_state determines what Timer 5 is doing
         * Trig pulses should be 10 microseconds long (5 ticks)
         * Distance = V*T/2; where V is the speed of sound (343.2m/s) and T is the Echo pulse length
         */ 
        
//        // Handle Front RF
//        if(RF_trigger_state == 2)
//        {
//            new_F_echo_state = _RB12;
//            if(new_F_echo_state != last_F_echo_state)
//            {
//                if(last_F_echo_state == 0)
//                    F_count = TMR5;
//                else
//                {
//                    last_F_range = F_range;
//                    new_F_range = (TMR5 - F_count)*count2range + F2L_DIST;
//                    F_range = (last_F_range + new_F_range)/2.0;     // digital signal averaging
//                }
//            }
//            last_F_echo_state = new_F_echo_state;
//        }
        
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
        
        // Handle Turret Tracking
        if(turret_tracking)
        {
            // Update the turret IR reading
            new_T_IR = ADC1BUF10;                   // Pin 17: AN10 (Analog Input) <-- Input from Left Base IR Sensor
            T_IR = (last_T_IR + new_T_IR)/2.0;      // Slight Digital Average Filtering

            if(game_time >= last_tracking_time + tracking_delay)
            {
                // Adjust the tracking direction of the turret
//                if(last_T_IR > T_IR)        // Invert Tracking direction if signal is becoming weaker
//                {
//                    tracking_step *= -1;
//                    switch_count =+ 1;
//                }
                if(turret_angle >= turret_max)       // don't turn out of the range of motion of the turret
                {
                    tracking_step = -1 * max_tracking_step;
                }
                if(turret_angle <= turret_min)       // don't turn out of the range of motion of the turret
                {
                    tracking_step = max_tracking_step;
                }
                turret_angle += tracking_step;
                last_tracking_time = game_time;
            }
            last_T_IR = new_L_IR;       // Set the new_T_IR reading to the last_IR readings for the next loop

        }
        
        // Set Angles of Servo Motors
        OC1R = turret_angle/1;
        OC3R = cannon_angle;
        
        // Set speed of stepper motors
        PR2 = track_speed;
        OC2RS = PR2;
        OC2R = PR2/2.0;

        // Set direction of stepper motors
        if(L_dir == 1)
            _LATB2 = 1;
        else
            _LATB2 = 0;
        if(R_dir == 1)
            _LATA2 = 0;
        else
            _LATA2 = 1;
        
        // Turn on/Off shooting motors
        if(cannon_motors)
        {
            _LATB9 = 1;
        }
        else
        {
            _LATB9 = 0;
        }
    }
    track_speed = 0;
    turret_tracking = 0;
    cannon_motors = 0;
    
return 0;
}

void config_ad(void)
{

    // AD1CHS register
    _CH0NA = 0;         // AD1CHS<7:5> -- Use VSS as negative input

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
     // Configure Timer 1 --> PWM signal to Turret Servo Motor
    _TON = 1;                   //enable Timer1
    _TCS = 0;                   //Set source to internal clock
    _TCKPS = 0b01;              //Select prescale value of 8:1 - Tick Period of 2 microseconds
    PR1 = 10000;                //Set timer frequency to 50 Hz
    TMR1 = 0;                   //Set timer count to 0
    // Configure Timer 2 --> PWM step signal to Stepper Motors
    T2CONbits.TON = 1;          //enable Timer2
    T2CONbits.TCS = 0;          //Set source to internal clock
    T2CONbits.TCKPS = 0b11;     //Select prescale value of 256:1 - Tick Period of 64 microseconds
    PR2 = 2;                    //Set initial timer period to 128 microseconds
    TMR2 = 0;                   //Set timer count to 0
    // Configure Timer 3 --> PWM signal to Cannon Feed Servo Motor
    T3CONbits.TON = 1;          //enable Timer3
    T3CONbits.TCS = 0;          //Set source to internal clock
    T3CONbits.TCKPS = 0b01;     //Select prescale value of 8:1 - Tick Period of 2 microseconds
    PR3 = 10000;                //Set timer frequency to 50 Hz
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
    OC1RS = turret_mid;             //Set duty cycle to the midpoint value
  // Configure Output Compare 2
    OC2CON1bits.OCTSEL = 0b000;     //Select Timer2 to be timer source
    OC2CON1bits.OCM = 0b110;        //Select Edge-Aligned PWM mode
    OC2CON2bits.SYNCSEL = 0b01100;  //Select Timer2 as synchronization source
    OC2RS = PR2/2;                  //Set duty cycle to 1/2 period
  // Configure Output Compare 3
    OC3CON1bits.OCTSEL = 0b001;     //Select Timer3 to be timer source
    OC3CON1bits.OCM = 0b110;        //Select Edge-Aligned PWM mode
    OC3CON2bits.SYNCSEL = 0b01101;  //Select Timer3 as synchronization source
    OC3RS = cannon_mid;             //Set duty cycle to the midpoint value
}

void config_IO(void)
{
 /*
 * Pin 9: AN15 RB4 (Analog Input) --> Input From Turret IR Sensor
 * Pin 17: AN10 RB14 (Analog Input) <-- Input from Left Base IR Sensor
 * Pin 18: AN9 RB15 (Analog Input) <-- Input from Right Base IR Sensor*/
    // Configure the digital I/O ports
    TRISA = 0;          //Set A ports to output
    TRISB = 0;          //Set B ports to output
    ANSA = 0;           //disables Port A analog input
    ANSB = 0;           //disables Port B analog input
    
    //Set input pins to input mode
    _TRISB4 = 1;
    _TRISB12 = 1;
    _TRISB13 = 1;
    _TRISB14 = 1;
    _TRISB15 = 1;
    
    //Enable Analog in read from analog in pins

    _ANSB4 = 1;
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
            _LATB7 = 1;
            RF_trigger_state = 1;
            TMR5 = 0;
            break;

        // State 1: time to drop Front RF_trigger and listen for Front RF echo
        case 1:
            PR5 = 50000;
            _LATB7 = 0;
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
            _LATB8 = 1;
            RF_trigger_state = 4;
            TMR5 = 0;
            break;

        // State 4: time to drop Lateral RF_trigger and listen for Lateral RF echo
        case 4:
            PR5 = 50000;
            _LATB8 = 0;
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

void __attribute__ ((__interrupt__)) _IC3Interrupt(void)
// Front US Rangefinder
{
    int C1 = 0; int C2 = 0; int C3 = 0; int C4 = 0; int C5 = 0; float IC_ave = 0;
    IFS2bits.IC3IF = 0; // Reset respective interrupt flag
    IC_count = IC3BUF; // Read and save off first entry
    C5 = C4;
    C4 = C3;
    C3 = C2;
    C2 = C1;
    C1 = IC_count;
    range_count += 1;
    if(range_count >= 5)
    {
        IC_ave = (C1 + C2 + C3 + C4 + C5)/5.0;
        F_range = IC_ave * count2range + F2L_DIST;
        range_count = 0;
    }
}

void config_IC(void)
{
    IC3CON2bits.SYNCSEL = 0b01110; // Timer 4 Sync Source
    IC3CON1bits.IC3TSEL = 0b010; // Use Timer 4
    IC3CON1bits.ICI = 0; // interrupt on every edge
    IC3CON1bits.ICM = 0b010; // Capture on falling edge
    IC3CON2bits.ICTRIG = 0; // Synchronizes ICx w/ source designated in SYNCSELx bits
    
    IFS2bits.IC3IF = 0; // Clear the IC3 interrupt status flag
    IEC2bits.IC3IE = 1; // Enable IC3 interrupts
    IPC9bits.IC3IP = 7; // Set module interrupt priority as 7
    
    IC_count = IC3BUF; // clear the buffer
    IC_count = IC3BUF;
    IC_count = IC3BUF;
    IC_count = IC3BUF;
}