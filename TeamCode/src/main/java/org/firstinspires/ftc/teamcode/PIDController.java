package org.firstinspires.ftc.teamcode;

public class PIDController {
    private final double kP;
    private final double kI;
    private final double kD;
    private final double maxIntegral;
    private double integral = 0.0;
    private double previousError = 0.0;
    private long previousTime;

    public PIDController(double kP, double kI, double kD, double maxIntegral) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.maxIntegral = maxIntegral;
        this.previousTime = System.currentTimeMillis();
    }

    public double calculate(double setpoint, double currentPosition) {
        long currentTime = System.currentTimeMillis();
        double error = setpoint - currentPosition;
        double dt = (currentTime - previousTime) / 1000.0; // convert ms to seconds

        // Proportional term
        double p = kP * error;

        // Integral term with anti-windup
        integral += kI * error * dt;
        if (integral > maxIntegral) {
            integral = maxIntegral;
        } else if (integral < -maxIntegral) {
            integral = -maxIntegral;
        }
        double i = integral;

        // Derivative term
        double d = 0.0;
        if (dt > 0) {
            d = kD * (error - previousError) / dt;
        }

        // Combine terms
        double output = p + i + d;

        // Save for next iteration
        previousError = error;
        previousTime = currentTime;

        return output;
    }

    public void reset() {
        integral = 0.0;
        previousError = 0.0;
        previousTime = System.currentTimeMillis();
    }
}