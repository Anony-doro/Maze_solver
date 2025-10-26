
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

void Bicycle::setSteerAngle(double steerCmd)
    {
      // Clamp the desired angle to valid range
      steerCmd = std::clamp(steerCmd, -deltaMax, deltaMax);
      
      // PD controller to reach the desired steering angle
      double error = steerCmd - delta;
      
      // Calculate error derivative
      // Note: Assumes constant dt between calls (typical dt = 0.05s)
      // For better accuracy, use setSteerAngleWithTime() with explicit time
      double errorDerivative = 0.0;
      
      if (!firstSteeringCall) {
          // Approximate derivative (assumes dt=0.05s between calls)
          errorDerivative = (error - lastSteeringError) / 0.05;
      }
      
      // PD control: u_rate_cmd = Kp * error + Kd * error_derivative
      double controlOutput = steeringKp * error + steeringKd * errorDerivative;
      
      // Store current error for next iteration
      lastSteeringError = error;
      firstSteeringCall = false;
      
      // Clamp the steering rate to maximum allowed rate
      u_rate_cmd = std::clamp(controlOutput, -steerRateMax, steerRateMax);
    }

void Bicycle::setSteerAngleWithTime(double steerCmd, double currentTime)
    {
      // Clamp the desired angle to valid range
      steerCmd = std::clamp(steerCmd, -deltaMax, deltaMax);
      
      // PD controller to reach the desired steering angle
      double error = steerCmd - delta;
      
      // Calculate error derivative with explicit time tracking
      double errorDerivative = 0.0;
      
      if (!firstSteeringCall) {
          double dt = currentTime - lastSteeringTime;
          if (dt > 0) {
              errorDerivative = (error - lastSteeringError) / dt;
          }
      }
      
      // PD control: u_rate_cmd = Kp * error + Kd * error_derivative
      double controlOutput = steeringKp * error + steeringKd * errorDerivative;
      
      // Store current state for next iteration
      lastSteeringError = error;
      lastSteeringTime = currentTime;
      firstSteeringCall = false;
      
      // Clamp the steering rate to maximum allowed rate
      u_rate_cmd = std::clamp(controlOutput, -steerRateMax, steerRateMax);
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
double Bicycle::getVelocity() const
    {
      return v;
    }

// Back-and-forth alignment control methods
void Bicycle::resetAlignmentState()
    {
      initialAngleDiff = 0.0;
      isBackAndForthMode = false;
      movingForward = true;
      lastAngleDiff = 0.0;
      backwardInitialAngle = 0.0;
    }

bool Bicycle::getIsBackAndForthMode() const
    {
      return isBackAndForthMode;
    }

void Bicycle::setIsBackAndForthMode(bool mode)
    {
      isBackAndForthMode = mode;
    }

bool Bicycle::getMovingForward() const
    {
      return movingForward;
    }

void Bicycle::setMovingForward(bool forward)
    {
      movingForward = forward;
    }

double Bicycle::getInitialAngleDiff() const
    {
      return initialAngleDiff;
    }

void Bicycle::setInitialAngleDiff(double diff)
    {
      initialAngleDiff = diff;
    }

bool Bicycle::getSwitchMode() const
    {
      return switchMode;
    }

void Bicycle::setSwitchMode(bool mode)
    {
      switchMode = mode;
    }

double Bicycle::getBackwardInitialAngle() const
    {
      return backwardInitialAngle;
    }

void Bicycle::setBackwardInitialAngle(double diff)
    {
      backwardInitialAngle = diff;
    }

void Bicycle::setSteeringControllerGains(double kp, double kd)
    {
      steeringKp = kp;
      steeringKd = kd;
    }

std::tuple<double, double> Bicycle::getSteeringControllerGains() const
    {
      return {steeringKp, steeringKd};
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
    
    // Normalize phi to [-π, π] range
    while (phi > M_PI) phi -= 2.0 * M_PI;
    while (phi < -M_PI) phi += 2.0 * M_PI;
}
