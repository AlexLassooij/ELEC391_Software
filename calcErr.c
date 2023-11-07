// within DAQ, acquires number of pulses counted and calculates error

double calcErr(double desired_angle, int n_pulses) {
    static double prev_angle;
    // amount of windows in sensor
    int n_windows = 100;
    // compensate gain, as sensor applies a "gain" to the output by only
    // counting n pulses (as opposed to the actual angle in degrees)
    // res = 360 / (2N), each windows has on/off
    double gain = 360 / (2 * n_windows);

    double angle = prev_angle + gain * n_pulses;
    
    double angle_rounded = round(angle*25/90) * 90/25;
    prev_angle = angle_rounded;

    return err = desired_angle - angle_rounded;

}
