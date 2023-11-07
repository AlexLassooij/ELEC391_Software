% within DAQ, acquires number of pulses counted and calculates error

function[err] = calcErr(desired_angle, n_pulses) 
    persistent prev_angle;
    % amount of windows in sensor
    n_windows = 100;
    % compensate gain, as sensor applies a "gain" to the output by only
    % counting n pulses (as opposed to the actual angle in degrees)
    % res = 360 / (4N), with 100 windows, N would be 25
    gain = 360 / n_windows;

    angle = prev_angle + gain * n_pulses;
    
    angle_rounded = round(angle*25/90,0) * 90/25;
    prev_angle = angle_rounded;

    err = desired_angle - angle_rounded;

end
