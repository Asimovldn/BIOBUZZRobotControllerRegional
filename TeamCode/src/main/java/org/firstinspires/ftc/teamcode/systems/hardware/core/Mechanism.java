package org.firstinspires.ftc.teamcode.systems.hardware.core;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Context;
import org.firstinspires.ftc.teamcode.util.Util;

public class Mechanism {
    public DcMotorEx motor;
    private double targetVelocity;
    private double TICKS_PER_DEGREE = 0;
    private HardwareMap hw;
    public Context runtime;

    private final Context.Constants constants = new Context.Constants();

    public Mechanism(double TICKS_PER_DEGREE) {
        this.TICKS_PER_DEGREE = TICKS_PER_DEGREE;
    }

    public void setup() {};

    public Mechanism() {
        this.TICKS_PER_DEGREE = Util.ticksPerDegree(28);
    }

    public void setTargetVelocity(double v) {
        targetVelocity = v;
    }

    public void setThrottle(double x) {
        motor.setPower(x);
    }

    public double Speed() {
        if (motor == null) return 0;
        return motor.getVelocity();
    }

    public boolean atSpeed(double target, double tolerance) {
        return Math.abs(Speed() / TICKS_PER_DEGREE - target) < tolerance;
    }

    public void periodic() {

    }

    /** Pass the runtime to the Mechanism, auto-handled by the RobotOpMode, do not use. */
    public void passInRuntime(Context ctx) {
        runtime = ctx;
    }
}
