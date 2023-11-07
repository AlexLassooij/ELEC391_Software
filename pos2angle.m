function [phi1, phi2] = pos2angle(dx, dy)
    if (sqrt(dx^2 + dy^2) > 250)
        % modify to return previous angles
        return;
    end
    la = 250;
    beta = acos(1-(dx^2+dy^2)/(2*la^2))
    delta = acos(sqrt(dx^2+dy^2)/(2*la));
    gamma = atan2(dy,dx);
    alpha = delta + gamma

    phi1 = pi / 2 - alpha;
    phi2 = pi * 3 / 2 - (alpha + beta);

    if (phi2 > pi)
        phi2 = phi2 - pi
    end

    if (phi1 > pi)
        phi1 = phi1 - pi
    end

    phi1deg = phi1 * 180 /pi
    phi2deg = phi2 * 180 / pi
    
    
end
