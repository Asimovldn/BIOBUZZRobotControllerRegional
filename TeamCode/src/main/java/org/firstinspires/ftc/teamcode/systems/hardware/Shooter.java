package org.firstinspires.ftc.teamcode.systems.hardware;

import static dev.nextftc.units.Units.Radians;
import static dev.nextftc.units.Units.RadiansPerSecond;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.storage.Storage;
import org.firstinspires.ftc.teamcode.systems.hardware.core.SimpleMotorized;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.units.measuretypes.AngularVelocity;

public class Shooter implements Mechanism {
    private NextMotor backmotor, foremotor;
    private AngularVelocity targetSpeed = RadiansPerSecond.of(0);
    private final Storage.ShooterConstants.BackShooter backConstants = new Storage.ShooterConstants.BackShooter();
    private final Storage.ShooterConstants.ForeShooter foreConstants = new Storage.ShooterConstants.ForeShooter();

    public Shooter() {
        backmotor = new NextMotor("foreShooter");
        foremotor = new NextMotor("backShooter");

        backmotor.setDirection(NextMotor.Direction.FORWARD);

        backmotor.getVelocityConstants().setKS(backConstants.kS);
        backmotor.getVelocityConstants().setKV(backConstants.kV);

        foremotor.getVelocityConstants().setKS(foreConstants.kS);
        foremotor.getVelocityConstants().setKV(foreConstants.kV);
    }

    public void setThrottle(double x) {
        backmotor.setThrottle(x); foremotor.setThrottle(x);
    }

    public void setTargetVelocity(AngularVelocity x) {
        targetSpeed = x;
    }

    /**
     * In Order:
     * [backMotor Speed, foreMotor Speed]
     * Speed()[0], Speed()[1]
     * @return
     */
    public AngularVelocity[] Speed() {
        AngularVelocity[] vels = new AngularVelocity[2];
        vels[0] = backmotor.getEncoderVelocity();
        vels[1] = foremotor.getEncoderVelocity();

        return vels;
    }

    public Command prepareLaunch(double strength) {
        return instant(() -> {
                foremotor.setVelocitySetpoint(targetSpeed);
        });
    }

    public Command enableLaunch() {
        return instant(() ->
                backmotor.setThrottle(1)
        );
    }

    public Command fullBrake() {
        return instant(() -> {
                foremotor.setVelocitySetpoint(RadiansPerSecond.of(0));
                backmotor.setVelocitySetpoint(RadiansPerSecond.of(0));
        });
    }

    @Override
    public void periodic() {
        backmotor.update();
        foremotor.update();
    }

    public Command disableLaunch() {
        return instant(() ->
                backmotor.setThrottle(0)
        );
    }
}
