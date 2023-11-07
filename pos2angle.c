#include <math.h>

 double * pos2angle(int dx, int dy) { 
    static double angles[2];
    // if position is impossible to reach, return previous values
    if (sqrt(dx^2 + dy^2) > 250) return angles;

    int la = 250;
    double beta = acos(1-(dx^2+dy^2)/(2*la^2));
    double delta = acos(sqrt(dx^2+dy^2)/(2*la));
    double gamma = atan2(dy,dx);
    double alpha = delta + gamma;

    double phi1 = pi / 2 - alpha;
    double phi2 = pi * 3 / 2 - (alpha + beta);

    // in case angles over 180 degrees is returned
    if (phi2 > pi)  phi2 = phi2 - pi;
    if (phi1 > pi) phi1 = phi1 - pi;
    if (phi2 < pi)  phi2 = phi2 + pi;
    if (phi1 < pi) phi1 = phi1 + pi;

    angles[0] = phi1;
    angles[1] = phi2;

    return angles;

    // phi1deg = phi1 * 180 /pi
    // phi2deg = phi2 * 180 / pi
 }
