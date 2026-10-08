package org.firstinspires.ftc.teamcode.systems.hardware.core;

import static dev.nextftc.units.Units.DegreesPerSecond;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.storage.Storage;

import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedback.PIDController;
import dev.nextftc.control.feedforward.SimpleFFCoefficients;
import dev.nextftc.control.feedforward.SimpleFeedforward;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.units.measuretypes.AngularVelocity;

/**
 * Class for all mechanism consisting of a motor that spins when commanded (Intake, Shooter, etc) inherits.<br>
 *
 * Consists of methods for {@code velocity} (with or without encoders), {@code periodic} with just a simple {@code motor.update()}.
 *
 * Example of use: <pre>{@code
 *     public Intake() {
 *          super("intake");
 *
 *          motor.getVelocityConstants.setKS(0.16);
 *          motor.getVelocityConstants.setKP(0.012);
 *
 *          alterConstants();
 *     }
 *
 *     // no other methods required for now.
 *     // to set power in, for example, teleop, do:
 *
 *     new Intake().setTargetVelocity(RotationsPerMinute.of(300));
 *     }
 *
 * @see dev.nextftc.robot.Mechanism
 */
public class SimpleMotorized implements Mechanism {

    /**
    THE MOTOR, {@code THE RULERRRRRRRRRRRRRR}.

     the whole reason of this class.

     @see NextMotor
     */
    private NextMotor motor;

    private PIDController velocityPID;
    private SimpleFeedforward velocityFF;

    public SimpleMotorized() {
        // motor = new NextMotor("motor");

        velocityPID = motor.getVelocityPID();
        velocityFF = motor.getVelocityFF();
    }

    /**
     * Call this constructor for automatic motor creation, then, if constants are edited, call {@code alterConstants()}.<br>
     * Example of use: <pre>{@code
     *     public Intake() {
     *          super("intake");
     *
     *          motor.getVelocityConstants.setKS(0.16);
     *          motor.getVelocityConstants.setKP(0.012);
     *
     *          alterConstants();
     *     }
     *
     *     // no other methods required for now.
     *     // to set power in, for example, teleop, do:
     *
     *     new Intake().setTargetVelocity(RotationsPerMinute.of(300));
     *     }
     *
     * @see dev.nextftc.robot.Mechanism
     */
    public SimpleMotorized(String motorName) {
        motor = new NextMotor(motorName);

        velocityPID = motor.getVelocityPID();
        velocityFF = motor.getVelocityFF();
    }

    /**
     * Manda o motor girar até uma certa velocidade.
     */
    public void setTargetVelocity(AngularVelocity v) {
        motor.setVelocitySetpoint(v);
    }

    /**
     * Updates the motor velocity coefficients.
     * @param pidCoeff new PID Coefficients.
     * @param ffCoeff new FF Coefficients.
     */
    public void alterConstants(PIDCoefficients pidCoeff, SimpleFFCoefficients ffCoeff) {
        motor.getVelocityConstants().setKP(pidCoeff.kP);
        motor.getVelocityConstants().setKI(pidCoeff.kI);
        motor.getVelocityConstants().setKD(pidCoeff.kD);
        motor.getVelocityConstants().setKV(ffCoeff.kV);
        motor.getVelocityConstants().setKS(ffCoeff.kS);
        motor.getVelocityConstants().setKA(ffCoeff.kA);

        velocityFF = motor.getVelocityFF();
        velocityPID = motor.getPositionPID();
    }

    /**
     * Updates the {@link #velocityFF} and {@link #velocityPID} properties after altering
     * constants outside of {@link #alterConstants(PIDCoefficients, SimpleFFCoefficients)}.
     * <br>Call this method after editing constants on your own.
     */
    public void alterConstants() {
        velocityFF = motor.getVelocityFF();
        velocityPID = motor.getPositionPID();
    }

    /**
     * Sets the motor throttle, dump of 10% added for safety {@code (x * 0.9)}.
     * @param x
     */
    public void setThrottle(double x) {
        motor.setThrottle(x * 0.9);
    }

    public AngularVelocity Speed() {
        return motor.getEncoderVelocity();
    }

    /**
     * Checks if motor is currently at {@code target} speed.
     * @param target target speed, is at this speed.
     * @param tolerance tolerance
     * @return boolean.
     */
    public boolean atSpeed(AngularVelocity target, AngularVelocity tolerance) {
        return Math.abs(Speed().into(DegreesPerSecond)
                - target.into(DegreesPerSecond)) < tolerance.into(DegreesPerSecond);
    }

    /**
     * Check if motor speed is EXACTLY at target.
     * @param target target speed.
     * @return boolean.
     */
    public boolean atSpeed(@NonNull AngularVelocity target) {
        return atSpeed(target, DegreesPerSecond.of(0));
    }

    @Override
    public void periodic() {
        motor.update();
    }
}
