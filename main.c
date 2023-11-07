#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* constants */

// Clock speed of Arduino Leonardo : 16MHz
const uint16_t CLK_FREQ = 16000000;
// Frequency of ISR : 100Hz
const uint16_t ISR_FREQ = 100;
const uint16_t TIMER_RESET = 0;
// compare match register = [ 16,000,000Hz/ (prescaler * desired interrupt frequency) ] - 1 --> 25 for 600Hz
const uint16_t COMP_MATCH = 25;

// PWM frequency should be at least 5 / (2pi *tau), where tau is how long 
// it takes the motor to reach its max voltage
// or T < (2pi * tau) / 5
// tau = L / R = 0.0263 * 10^-3 / 0.0788 = 0.333ms
// 5 / (2pi * tau) = 2,390 Hz 
// motor acts like a low pass filter
// standard leonardo PWM Freq : 490Hz (T = 2.04ms)
// default TOP value is 255, can change it with OCRnA register
// default prescaler 64, so freq is 16MHz / (256 * 64) / 2 ~ 490Hz (divide by 2 since pin cant be toggled on and off in same cycle)
// we need the PWM frequency to be > 2390, so we will change to prescaler from 64 to 8 by setting the bits of the 
// TCCR0A register. Advised not to change this since timer 0 is used for delay and milli func, but not used in our application
// will not interfere with ISR either since it uses Timer 1.
// pins 3 and 11 are 980 Hz

// Forward and reverse analog out for motors
const uint16_t M1_DRIVE_PIN_F = 3;
const uint16_t M2_DRIVE_PIN_F = 5;
const uint16_t M1_DRIVE_PIN_R = 6;
const uint16_t M2_DRIVE_PIN_R = 9;

const uint16_t ENCODER_UP_DN_J1 = 10;
const uint16_t ENCODER_UP_DN_J2 = 12;

// external interrupt 2 on pin 0
const uint16_t ENCODER_IN_J1 = 0;
// external interrupt 4 on pin 7
const uint16_t ENCODER_IN_J2 = 7;

// digital input pin for homing sensor
const uint16_t HOMING_IN = 8;

// minimum return value from analogRead() to be considered as high (analogRead returns 0-1024 mapped to 0-5 Volts)
const uint16_t MIN_LOGIC_HIGH = 765;



// maximum acceptable error (rad)
const double MAX_ERR = 0.025;
const int MAX_VOLTAGE = 12;
const int MAX_PWM_DC = 255;


const double dt = 1 / ISR_FREQ;

const uint16_t NWINDOWS = 100;

// arm length in mm
const double ARM_LENGTH_1 = 125;
const double ARM_LENGTH_2 = 125;


// weights for weighted sum calculation of latest output angles
const double c1 = 0.09;
const double c2 = 0.245;
const double c3 = 0.665;

const double pi = 3.1415926536;

// Gain to map radians to voltage
const double Rad2VGain = MAX_VOLTAGE / (pi / 2);

// PID gains for PID control of each joint
const double K1 = 1 , Kp1 = 1, Ki1 = 0, Kd1 = 0, K2 = 1, Kp2 = 1, Ki2 = 0, Kd2 = 0;

const uint16_t NUM_TARGETS = 4;

/* 5 global variable declarations */
 
double target_list[NUM_TARGETS][2];

// index to keep track of what point is being travelled to
int current_dest = 0;
// initialize empty array for desired and current angles
double desired_angles[2] = {0};
double current_angles[2] = {0};
double integral[2] = {0};


// populate with first desired angles

// set initial errors
double error[2][3] = {0};
double output_angles[2] = {0};

// accumulator for counted pulses
int pulses[2][3] = {0};

// homing flag
int IS_HOME = 0;

/* 6 function prototypes */

void find_home();
void calcErr(int joint);
void pos2angle();
void PID_control(int joint);
double applyVGain(double output_angle);
void set_drive();

// reference to ATmega32u4 datasheet
// int main(int argc, char *argv[]) {

void setup()
    // initialize target list of sample x- and y-coordinates : 
    target_list[0][0] = 100;
    target_list[0][1] = 100;

    target_list[1][0] = 150;
    target_list[1][1] = 50;

    target_list[2][0] = 100;
    target_list[2][1] = -100;

    target_list[3][0] = -100;
    target_list[3][1] = -150;
    pos2angle();
    error = {
        {desired_angles[0], 0, 0},
        {desired_angles[1], 0, 0}
        };
    pinMode(HOMING_IN, INPUT);
    pinMode(ENCODER_IN_J1, INPUT_PULLUP);
    pinMode(ENCODER_UP_DN_J1, INPUT_PULLUP);
    pinMode(ENCODER_IN_J2, INPUT_PULLUP);
    pinMode(ENCODER_UP_DN_J2, INPUT_PULLUP);

    pinMode(M1_DRIVE_PIN_F, OUTPUT);
    pinMode(M1_DRIVE_PIN_R, OUTPUT);
    pinMode(M2_DRIVE_PIN_F, OUTPUT);
    pinMode(M2_DRIVE_PIN_R, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(ENCODER_IN_J1), readEncoder1, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_IN_J2), readEncoder2, RISING);

    // Reset Timer1 control
    TCCR1A = 0;
    // We want the interrupt to trigger 600Hz frequency
    // Clock frequency is 16MHz, and the max prescaler built into the timers is 1024
    // this means that we would need to set the compare match register as well
    // Set prescaler -> set registers to 100, which is 4 in decimal
    TCCR1B |= (1 << CS12);
    TCCR1B &= ~(1 << CS11);
    TCCR1B &= ~(1 << CS10);

    // Change timer 0 prescaler to 8 to achieve sufficient PWM frequency (010) in bits CS02-00
    // PWM frequency will be 16Mhz / (2 * 256 * prescaler) = 3,906.25Hz
    TCCR0B &= ~(1 << CS12);
    TCCR0B |= ~(1 << CS11);
    TCCR0B &= ~(1 << CS10);

    // Reset timer1
    TCNT1 = TIMER_RESET;
    // Set compare match register
    OCR1A = COMP_MATCH;
}

void loop() {
    // keep CPU busy
    // actual delay is 125ms since timer0 prescaler was changed
    delay(1000);

    // start homing if not done yet
    if(!IS_HOME) {
        find_home();
        // Enable Timer1 compare interrupt
        TIMSK1 = (1 << OCIE1A);
        // enable global interrupts 
        sei();
    }
    // if errors are below accepted error, that means the robot has reached the desired position
    // and is ready to move to the next position
    if (error[0] < MAX_ERR && error[1] < MAX_ERR) {
        // terminate operation when last destination has been reached
        if (!target_list[++current_dest]) {
            exit(0);
        }

        // disable interrupts during new angle computations
        TIMSK1 &= ~(1 << OCIE1A);
        // index to keep track of what point is being travelled to
        current_dest++;
        
        pos2angle();
        // set initial error relative to previous angle
        error[0] = desired_angles[0] - current_angles[0] ;
        error[1] = desired_angles[1] - current_angles[1] ;

        // re-enable interrupts
        TCNT1 = TIMER_RESET;
        TIMSK1 = (1 << OCIE1A);
    }

}



// TIMER1_COMPA vector at address $0022 according to datasheet, p.63
// TODO in interrupt
// count pulses from sensor
// calculate new error
// drive motor in proper direction
ISR(TIMER1_COMPA_vect) {
    TCNT1 = TIMER_RESET;
    // calculate new error signal
    calcErr(1);
    calcErr(2);
    // process error signal to produce control variable
    PID_control(1);
    PID_control(2);
    // drive motors
    set_drive();
}

/* 8 function declarations */

void find_home() {
    // pin will only go high when both joints are in "home" position,
    // where they will block the photo sensor, causing the NAND gate to output logic high 
    while(digitalRead(HOMING_IN == LOW)) {
        
        delay(2000);
    }

    IS_HOME = 1;
}

void set_drive() {

    double V1 = applyVGain(output_angles[0]);
    double V2 = applyVGain(output_angles[1]);

    // set duty cycle to achieve desired voltage
    int DC1 = round(V1 / MAX_VOLTAGE * MAX_PWM_DC);
    int DC2 = round(V2 / MAX_VOLTAGE * MAX_PWM_DC);

    if(V1 >= 0 && V2 >= 0) {
        analogWrite(M1_DRIVE_PIN_F, DC1);
        analogWrite(M2_DRIVE_PIN_F, DC2);
        analogWrite(M1_DRIVE_PIN_R, 0);
        analogWrite(M2_DRIVE_PIN_R, 0);
    }
    else if(V1 >= 0 && V2 < 0) {
        analogWrite(M1_DRIVE_PIN_F, DC1);
        analogWrite(M2_DRIVE_PIN_F, 0);
        analogWrite(M1_DRIVE_PIN_R, 0);
        analogWrite(M2_DRIVE_PIN_R, DC2);
    }
    else if(V1 <0 && V2 >= 0) {
        analogWrite(M1_DRIVE_PIN_F, 0);
        analogWrite(M2_DRIVE_PIN_F, DC2);
        analogWrite(M1_DRIVE_PIN_R, DC1);
        analogWrite(M1_DRIVE_PIN_R, 0);
    }
    else {
        analogWrite(M1_DRIVE_PIN_F, 0);
        analogWrite(M2_DRIVE_PIN_F, 0);
        analogWrite(M1_DRIVE_PIN_R, DC1);
        analogWrite(M2_DRIVE_PIN_R, DC2);
    }
}

double applyVGain(double output_angle) {
    if (output_angle >= 0) {
        return (output_angle * Rad2VGain > 12) ? 12 : output_angle * Rad2VGain;
    }
    else {
        return (output_angle * Rad2VGain) < -12 ? -12 : output_angle * Rad2VGain;
    }
}

void calcErr(int joint) {
    // n_pulses : signed integer
    // compensate gain, as sensor applies a "gain" to the output by only
    // counting n pulses (as opposed to the actual angle in degrees)
    // res = 360 / (2N), each windows has on/off
    double gain = 360 / (2 * NWINDOWS);

    double weighted_pulses = c1 * pulses[joint - 1][0] + c2 * pulses[joint - 1][1] + c3 * pulses[joint - 1][2];

    current_angles[joint - 1] += gain * weighted_pulses;
    error[joint - 1][0] = desired_angles[joint - 1] - current_angles[joint - 1];

    // shift previous pulse counts for next calculation
    pulses[joint - 1][2] = pulses[joint - 1][1];
    pulses[joint - 1][1] = pulses[joint - 1][0];
    pulses[joint - 1][0] = 0;
}

void PID_control(int joint) {
    integral[joint - 1] += (error[joint - 1][0] + error[joint - 1][1]) / 2 * dt;
    double diff_err = c1 * error[joint - 1][0] + c2 * error[joint - 1][1] + c3 * error[joint - 1][2];
    double diff = (diff_err + c2 * error[joint - 1][1]) / dt;
    error[joint - 1][2] = error[joint - 1][1];
    error[joint - 1][1] = error[joint - 1][0];

    // weighted sum with n hat = 3, sum of 3 values
    if (joint == 1) {
        output_angles[0] = K1 * (Kp1 * error[0][0] + Ki1 * integral[0] + Kd1 * diff);
    }
    else {
        output_angles[1] = K2 * (Kp2 * error[1][0] + Ki2 * integral[1] + Kd2 * diff);
    }
}
    

// will initially try n hat set to 3, will experiment with different values
// for best results

 void pos2angle() { 
    double dx = target_list[current_dest][0];
    double dy = target_list[current_dest][1];
    // if position is impossible to reach, return do nothing
    if (sqrt(dx^2 + dy^2) > (ARM_LENGTH_1 + ARM_LENGTH_2)) return;

    double beta = acos((ARM_LENGTH_1^2+ARM_LENGTH_2^2-(dx^2+dy^2)) / (2*ARM_LENGTH_1*ARM_LENGTH_2);)
    double delta = acos((ARM_LENGTH_1^2 - ARM_LENGTH_2^2 + (dx^2+dy^2)) / (2*ARM_LENGTH_1*sqrt((dx^2+dy^2))));

    double gamma = atan2(dy, dx);
    double alpha = delta + gamma;

    desired_angles[0]  = pi / 2 - alpha;
    desired_angles[1] = pi - beta;
    return;
 }

 void readEncoder1 {
    pulses1[0][0] = (digitalRead(ENCODER_UP_DN_J1) == HIGH) ? pulses1[0][0] + 1 : pulses1[0][0] - 1;
 }

 void readEncoder2 { 
    pulses2[1][0] = (digitalRead(ENCODER_UP_DN_J2) == HIGH ?) pulses2[1][0] + 1 : pulses2[1][0] - 1;
 }

//  general notes : 

//     analogRead(PIN_NO);
//     analogread has 0-1024, PWM 0-255
