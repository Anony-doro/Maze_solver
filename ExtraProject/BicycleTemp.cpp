
#include "BicycleTemp.hpp"
#include <algorithm> // std::clamp
#include <cmath>

Bicycle::Bicycle(double x, double y, double phi)
    : x(x), y(y), phi(phi) {}

void Bicycle::setVelocity(double vMps)
    {
      v = vMps;
    }
void Bicycle::setSteerCmd(double uRadps)
    {
      u_rate_cmd = uRadps;
    }

void Bicycle::setPosition(double x, double y, double phi)
    {
      this->x = x;
      this->y = y;
      this->phi = phi;
    }

void Bicycle::setWheelBase(double wheelBaseMeter)
    {
      wheelBase = wheelBaseMeter;
    }
void Bicycle::setCgRatio(double cg_ratio)
    {
      centerGravity = cg_ratio;
    }

std::tuple<double,double,double> Bicycle::pose() const
    {
      return {x, y, phi};
    }

double Bicycle::steering() const
    {
      return delta;
    }
double Bicycle::sideslip() const
    {
      return beta;
    }

void Bicycle::step(double dt) { //we can set dt to 0.05 seconds
    // Rate-limit steering command
    double u = std::clamp(u_rate_cmd, -steerRateMax, steerRateMax);
    delta += u * dt;
    delta = std::clamp(delta, -deltaMax, deltaMax);

    // Instantaneous radius; guard delta ~ 0
    const double instRadius = std::abs(std::tan(delta)) > 1e-9 ? (wheelBase / std::tan(delta)) : 1e12;
    const double omega = v / instRadius;  // yaw rate

    // Simple bicycle sideslip approximation:
    // beta ≈ atan(l_r/L * tan(delta)), where cg_ratio_ ~= l_r/L
    beta = std::atan(centerGravity * std::tan(delta));

    x   += v * std::cos(phi + beta) * dt;
    y   += v * std::sin(phi + beta) * dt;
    phi += omega * dt;
}
