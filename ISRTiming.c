#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

/* constants */

// Clock speed of Arduino Leonardo : 16MHz
// Frequency of ISR : 100Hz
const uint16_t ISR_FREQ = 100;
const uint16_t TIMER_RESET = 0;
// compare match register = [ 16,000,000Hz/ (prescaler * desired interrupt frequency) ] - 1 --> 624 for 100Hz
const uint16_t COMP_MATCH = 624;

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


// maximum acceptable error (rad)
const double MAX_ERR = 0.025;
const int MAX_VOLTAGE = 12;
const int MAX_PWM_DC = 255;


const double dt = 1 / ISR_FREQ;

const uint16_t NWINDOWS = 100;

// arm length in mm
const double ARM_LENGTH = 250;

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
double error[2][3] = {
        {1, 2, 3},
        {1, 2, 3}
        };
    
double output_angles[2] = {0};

// accumulator for counted pulses
int pulses[2][3] = {
    {1, 2, 3},
    {1, 2, 3}
    };

// homing flag

/* 6 function prototypes */

void find_home();
void calcErr(int joint);
void pos2angle();
void PID_control(int joint);
double applyVGain(double output_angle);
void set_drive();
void ISR();

// reference to ATmega32u4 datasheet
// int main(int argc, char *argv[]) {

int main(void) {
    clock_t begin = clock();
    ISR();
    clock_t end = clock();

    printf("Elapsed: %f seconds\n", (double)(end - begin) / CLOCKS_PER_SEC);
    printf("dafuq");
}

    
  




// TIMER1_COMPA vector at address $0022 according to datasheet, p.63
// TODO in interrupt
// count pulses from sensor
// calculate new error
// drive motor in proper direction
void ISR() {
    // calculate new error signal
    calcErr(1);
    calcErr(2);

    // process error signal to produce control variable
    PID_control(1);
    PID_control(2);
    set_drive();
}

/* 8 function declarations */

void set_drive() {

    double V1 = applyVGain(output_angles[0]);
    double V2 = applyVGain(output_angles[1]);

    // set duty cycle to achieve desired voltage
    int DC1 = round(V1 / MAX_VOLTAGE * MAX_PWM_DC);
    int DC2 = round(V2 / MAX_VOLTAGE * MAX_PWM_DC);

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
    static double integ;

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

    // in case angles over 180 degrees are returned
    // if (phi2 > pi)  phi2 = phi2 - pi;
    // if (phi1 > pi) phi1 = phi1 - pi;
    // if (phi2 < pi)  phi2 = phi2 + pi;
    // if (phi1 < pi) phi1 = phi1 + pi;

    

    
    // phi1deg = phi1 * 180 /pi
    // phi2deg = phi2 * 180 / pi
 

//  general notes : 

//     analogRead(PIN_NO);
//     analogread has 0-1024, PWM 0-255
