double output_angle(double err) {
    int dt = 1;
    double kp = 1;
    double ki = 1;
    double kd = 1;
    double c1 = 1 / 9;
    double c2 = 3 / 9;
    double c3= 5 / 9;
    static double prev_err ;
    static double prev_angle;
    static double curr_angle;
    static double prev_prev_angle;
    static double integ;

    integ = integ + (err + prev_err) / 2 * dt;
    diff = (err + prev_err) / dt;
    prev_err = err;

    // weighted sum with n hat = 3, sum of 3 values
    prev_prev_angle = prev_angle;
    prev_angle = curr_angle;
    curr_angle = kp * err + kd * diff + ki * integ;
    double new_angle = c3 * curr_angle + c2 * prev_angle + c1 * prev_prev_angle;
}

// will initially try n hat set to 3, will experiment with different values
// for best results
