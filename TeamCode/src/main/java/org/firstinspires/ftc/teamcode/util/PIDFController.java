package org.firstinspires.ftc.teamcode.util;

public class PIDFController {
    private double kP, kI, kD, kF;

    private double integralSum = 0;
    private double lastError = 0;
    private double filteredDerivative = 0;
    private double integralClamp = Double.POSITIVE_INFINITY;
    private static final double DERIVATIVE_FILTER_ALPHA = 0.2; // 0 = no smoothing, 1 = frozen

    private boolean continuousInput = false;
    private double inputMin, inputMax;

    private long lastTimeNanos = -1;

    public PIDFController(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    /** Lets error wrap around, example: enableContinuousInput(-180, 180) for a turret heading. */
    public void enableContinuousInput(double minInput, double maxInput) {
        continuousInput = true;
        inputMin = minInput;
        inputMax = maxInput;
    }

    public void setIntegralClamp(double clamp) {
        this.integralClamp = Math.abs(clamp);
    }

    public void setGains(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    public void reset() {
        integralSum = 0;
        lastError = 0;
        filteredDerivative = 0;
        lastTimeNanos = -1;
    }

    /**
     * @param error        setpoint - measurement (raw; wrapping is handled internally if enabled)
     * @param feedforward  disturbance/feedforward input, multiplied by kF (ex: -botAngularVelocity)
     * @return controller output (ex: a target angular velocity)
     */
    public double calculate(double error, double feedforward) {
        long now = System.nanoTime();
        double dt = (lastTimeNanos < 0) ? 0.02 : (now - lastTimeNanos) / 1e9;
        lastTimeNanos = now;
        if (dt <= 0) dt = 1e-3;

        if (continuousInput) {
            double range = inputMax - inputMin;
            error = ((error - inputMin) % range + range) % range + inputMin;
            if (error > (range / 2 + inputMin)) error -= range;
        }

        integralSum += error * dt;
        integralSum = clamp(integralSum, integralClamp);

        double rawDerivative = (error - lastError) / dt;
        filteredDerivative += DERIVATIVE_FILTER_ALPHA * (rawDerivative - filteredDerivative);
        lastError = error;

        return kP * error + kI * integralSum + kD * filteredDerivative + kF * feedforward;
    }

    public double calculate(double error) {
        return calculate(error, 0);
    }

    private static double clamp(double v, double max) {
        if (v > max) return max;
        if (v < -max) return -max;
        return v;
    }

    public void setPIDF(double posKp, double posKi, double posKd, double posKf) {
        kP = posKp; kI = posKi; kD = posKd; kF = posKf;
    }
}