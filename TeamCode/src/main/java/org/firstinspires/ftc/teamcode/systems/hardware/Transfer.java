package org.firstinspires.ftc.teamcode.systems.hardware;

import static dev.nextftc.units.Units.DegreesPerSecond;

import org.firstinspires.ftc.teamcode.storage.Storage;
import org.firstinspires.ftc.teamcode.systems.hardware.core.SimpleMotorized;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.units.measuretypes.AngularVelocity;

public class Transfer implements Mechanism {
    private NextMotor motor;

    public Transfer() {
        motor = new NextMotor("transfer");

        motor.getVelocityConstants().setKS(0.16);
        motor.getVelocityConstants().setKV(0.00000517);
    }

    public void setTargetVelocity(AngularVelocity v) {
        motor.setVelocitySetpoint(v);
    }

    public void setThrottle(double x) {
        motor.setThrottle(x);
    }

    @Override
    public void periodic() {
        motor.update();
    }
}
