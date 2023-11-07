function [new_angle] = output_angle(err)
    dt = 1;
    kp = 1;
    ki = 1;
    kd = 1;
    c1 = 1/9;
    c2 = 3/9;
    c3= 5/9;
    persistent prev_err 
    persistent prev_angle
    persistent curr_angle
    persistent prev_prev_angle
    persistent integ

    integ = integ + (err + prev_err) / 2 * dt;
    diff = (err + prev_err) / dt;
    prev_err = err;

    % weighted sum with n hat = 3
    prev_prev_angle = prev_angle;
    prev_angle = curr_angle;
    curr_angle = kp * err + kd * diff + ki * integ;
    new_angle = c3 * curr_angle + c2 * prev_angle + c1 * prev_prev_angle;
end

% will initially try n hat set to 3, will experiment with different values
% for best results
