package org.firstinspires.ftc.teamcode.systems.hardware;

import org.firstinspires.ftc.teamcode.storage.Storage;
import org.firstinspires.ftc.teamcode.systems.hardware.core.SimpleMotorized;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.units.measuretypes.AngularVelocity;

public class Shooter implements Mechanism {
    private NextMotor motor;
    private final Storage.ShooterConstants constants = new Storage.ShooterConstants();

    public Shooter() {
        motor = new NextMotor("shooter");

        motor.getVelocityConstants().setKS(constants.kS);
        motor.getVelocityConstants().setKV(constants.kV);
    }

    public void setThrottle(double x) {
        motor.setThrottle(x);
    }

    public void setTargetVelocity(AngularVelocity x) {
        motor.setVelocitySetpoint(x);
    }

    public AngularVelocity Speed() {
        return motor.getEncoderVelocity();
    }

    @Override
    public void periodic() {
        motor.update();
    }
}
