//
// Created by andoro on 2025-10-14.
//

#ifndef BICYLCETEMP_HPP
#define BICYLCETEMP_HPP
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <tuple>
#include <cmath>

class Bicycle {
private:
    // state
    double x, y, phi;  // position, heading
    double beta = 0.0;   // sideslip
    double delta = 0.0;  // steering angle

    // params
    double wheelBase = 3.32;     // wheelbase [m]
    double centerGravity = 0.5;     // ratio for current CG
    double deltaMax = 35.0 * M_PI / 180.0; // rad
    double steerRateMax = 25.0 * M_PI / 180.0; // rad/s

    // inputs
    double v = 1.0;      // m/s
    double u_rate_cmd = 0.0; // desired steering rate (rad/s)

public:
    // All angles in radians
    Bicycle(double x, double y, double phi);

    // Control inputs
    void setVelocity(double vMps);           // linear speed velocity meterpersecond
    void setSteerCmd(double uRadps);         // steering rate command (rad/s)
    void setPosition(double x, double y, double phi); // set position directly

    // Integrate one step
    void step(double dt);

    // Read state
    std::tuple<double,double,double> pose() const; // x, y, phi
    double steering() const;                        // delta
    double sideslip() const;                        // beta

    // Vehicle params (you can expose setters if you need to tune at runtime)
    void setWheelBase(double wheelBaseMeter);
    void setCgRatio(double cg_ratio);              // ~lr/L (0..1)
};

#endif //BICYLCETEMP_HPP
