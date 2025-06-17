#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <chrono>
#include <algorithm>
#include <cmath>

/**
 * @brief A comprehensive PID controller implementation for robot control applications
 * 
 * This class provides a robust PID controller with:
 * - Configurable gains (Kp, Ki, Kd)
 * - Integral windup protection
 * - Output saturation limits
 * - Derivative kick prevention
 * - Reset functionality
 */
class PIDController {
public:
    /**
     * @brief Constructor for PID controller
     * @param kp Proportional gain
     * @param ki Integral gain  
     * @param kd Derivative gain
     * @param output_min Minimum output value (saturation limit)
     * @param output_max Maximum output value (saturation limit)
     * @param integral_max Maximum integral term (anti-windup)
     */
    PIDController(double kp, double ki, double kd, 
                  double output_min = -1.0, double output_max = 1.0,
                  double integral_max = 1.0)
        : kp_(kp), ki_(ki), kd_(kd),
          output_min_(output_min), output_max_(output_max),
          integral_max_(integral_max),
          previous_error_(0.0), integral_(0.0),
          first_call_(true) {}

    /**
     * @brief Compute PID output
     * @param setpoint Desired value
     * @param measurement Current measured value
     * @param dt Time step in seconds
     * @return PID controller output
     */
    double compute(double setpoint, double measurement, double dt) {
        // Calculate error
        double error = setpoint - measurement;
        
        // Proportional term
        double proportional = kp_ * error;
        
        // Integral term with anti-windup
        if (dt > 0.0) {
            integral_ += error * dt;
            // Clamp integral to prevent windup
            integral_ = std::clamp(integral_, -integral_max_, integral_max_);
        }
        double integral_term = ki_ * integral_;
        
        // Derivative term (with derivative kick prevention)
        double derivative = 0.0;
        if (!first_call_ && dt > 0.0) {
            // Use derivative of measurement instead of error to prevent derivative kick
            derivative = kd_ * (previous_error_ - error) / dt;
        }
        
        // Compute total output
        double output = proportional + integral_term + derivative;
        
        // Apply output saturation
        output = std::clamp(output, output_min_, output_max_);
        
        // Store for next iteration
        previous_error_ = error;
        first_call_ = false;
        
        return output;
    }
    
    /**
     * @brief Reset the PID controller state
     */
    void reset() {
        previous_error_ = 0.0;
        integral_ = 0.0;
        first_call_ = true;
    }
    
    /**
     * @brief Update PID gains during runtime
     */
    void setGains(double kp, double ki, double kd) {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }
    
    /**
     * @brief Update output limits during runtime
     */
    void setOutputLimits(double output_min, double output_max) {
        output_min_ = output_min;
        output_max_ = output_max;
    }
    
    /**
     * @brief Get current error value
     */
    double getError() const { return previous_error_; }
    
    /**
     * @brief Get current integral term
     */
    double getIntegral() const { return integral_; }
    
    /**
     * @brief Get individual PID components for debugging
     */
    struct PIDComponents {
        double proportional;
        double integral;
        double derivative;
        double total;
    };
    
    PIDComponents getLastComponents() const {
        return last_components_;
    }
    
    /**
     * @brief Compute PID with component tracking for debugging
     */
    double computeWithComponents(double setpoint, double measurement, double dt) {
        double error = setpoint - measurement;
        
        // Proportional term
        last_components_.proportional = kp_ * error;
        
        // Integral term with anti-windup
        if (dt > 0.0) {
            integral_ += error * dt;
            integral_ = std::clamp(integral_, -integral_max_, integral_max_);
        }
        last_components_.integral = ki_ * integral_;
        
        // Derivative term
        last_components_.derivative = 0.0;
        if (!first_call_ && dt > 0.0) {
            last_components_.derivative = kd_ * (previous_error_ - error) / dt;
        }
        
        // Total output
        last_components_.total = last_components_.proportional + 
                                last_components_.integral + 
                                last_components_.derivative;
        
        // Apply saturation
        double output = std::clamp(last_components_.total, output_min_, output_max_);
        
        // Store for next iteration
        previous_error_ = error;
        first_call_ = false;
        
        return output;
    }

private:
    // PID gains
    double kp_, ki_, kd_;
    
    // Output limits
    double output_min_, output_max_;
    double integral_max_;
    
    // Internal state
    double previous_error_;
    double integral_;
    bool first_call_;
    
    // For debugging
    mutable PIDComponents last_components_;
};

/**
 * @brief Specialized PID controller for robot angular velocity control
 * 
 * This class extends the basic PID controller with robot-specific features:
 * - Angle wrapping for circular setpoints
 * - Deadband for reducing oscillation near setpoint
 * - Velocity feedforward for smooth motion
 */
class AngularPIDController : public PIDController {
public:
    AngularPIDController(double kp, double ki, double kd,
                        double output_min = -2.0, double output_max = 2.0,
                        double deadband = 0.05, double feedforward = 0.0)
        : PIDController(kp, ki, kd, output_min, output_max),
          deadband_(deadband), feedforward_(feedforward) {}
    
    /**
     * @brief Compute angular PID with angle wrapping
     * @param setpoint_angle Target angle in radians
     * @param current_angle Current angle in radians  
     * @param dt Time step in seconds
     * @return Angular velocity command
     */
    double computeAngular(double setpoint_angle, double current_angle, double dt) {
        // Normalize angles to [-π, π]
        double error = normalizeAngle(setpoint_angle - current_angle);
        
        // Apply deadband to reduce oscillation
        if (std::abs(error) < deadband_) {
            reset();  // Reset integral when in deadband
            return 0.0;
        }
        
        // Use the base PID with normalized error
        double setpoint_normalized = current_angle + error;
        return compute(setpoint_normalized, current_angle, dt) + feedforward_;
    }
    
    void setDeadband(double deadband) { deadband_ = deadband; }
    void setFeedforward(double feedforward) { feedforward_ = feedforward; }

private:
    double deadband_;
    double feedforward_;
    
    /**
     * @brief Normalize angle to [-π, π] range
     */
    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
};

#endif // PID_CONTROLLER_HPP 