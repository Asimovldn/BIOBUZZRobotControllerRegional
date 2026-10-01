package org.firstinspires.ftc.teamcode.systems.hardware;

import static dev.nextftc.units.Units.Degrees;
import static dev.nextftc.units.Units.DegreesPerSecond;

import org.firstinspires.ftc.teamcode.storage.Storage;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.units.measuretypes.AngularVelocity;

public class Shooter implements Mechanism {
    private NextMotor motor;
    private final Storage.ShooterConstants constants = new Storage.ShooterConstants();

    public Shooter() {
        motor = new NextMotor("shooter", Degrees.of(360.0 / 28));

        motor.getVelocityConstants().setKS(constants.kS);
        motor.getVelocityConstants().setKV(constants.kV);
    }

    public void setTargetVelocity(AngularVelocity v) {
        motor.setVelocitySetpoint(v);
    }

    public void setThrottle(double x) {
        motor.setThrottle(x * 0.9);
    }

    public AngularVelocity Speed() {
        return motor.getEncoderVelocity();
    }

    public boolean atSpeed(AngularVelocity target, AngularVelocity tolerance) {
        return Math.abs(Speed().into(DegreesPerSecond)
                - target.into(DegreesPerSecond)) < tolerance.into(DegreesPerSecond);
    }

    @Override
    public void periodic() {
        motor.update();
    }
}
