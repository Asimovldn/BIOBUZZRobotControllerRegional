package org.firstinspires.ftc.teamcode.systems.hardware.core;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Context;

public class Mechanism {
    public DcMotorEx motor;
    protected double targetVelocity;
    public Context runtime;

    public void setup() {}

    /** Target in encoder ticks per second. */
    public void setTargetVelocity(double v) {
        targetVelocity = v;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public void setThrottle(double x) {
        if (motor != null) motor.setPower(x);
    }

    /** Encoder ticks per second. */
    public double Speed() {
        if (motor == null) return 0;
        return motor.getVelocity();
    }

    public boolean atSpeed(double target, double tolerance) {
        return Math.abs(Speed() - target) < tolerance;
    }

    public void periodic() {}

    /** Pass the runtime to the Mechanism, auto-handled by the RobotOpMode, do not use. */
    public void passInRuntime(Context ctx) {
        runtime = ctx;
    }
}
