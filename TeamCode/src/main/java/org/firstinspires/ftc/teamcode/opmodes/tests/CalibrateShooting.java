package org.firstinspires.ftc.teamcode.opmodes.tests;

import static dev.nextftc.units.Units.RadiansPerSecond;
import static dev.nextftc.units.Units.RotationsPerMinute;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import org.firstinspires.ftc.teamcode.systems.Bot;
import org.firstinspires.ftc.teamcode.systems.hardware.Turret;

import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;

@NextTeleop
public class CalibrateShooting extends NextOpMode {
    public Bot robot;
    public CommandGamepad gmp;
    public double strength = 4.8;
    public CalibrateShooting(Bot bot) {
        super(bot);
        robot = bot;

        gmp = new CommandGamepad(gamepad1);
    }

    @Override
    public void start() {
        Scheduler.reset();
        robot.getTurret().setMode(Turret.Mode.AIMING);
        robot.getTurret().reset();
    }

    @Override
    public void periodic() {
        telemetry.addData("Current Velocity: ", robot.getIntake().Speed().into(RotationsPerMinute));
        telemetry.addData("Current Angle: ", robot.getTurret().getAngle());
        telemetry.addData("Current Err: ", robot.getTurret().getError());
        telemetry.addData("Target Angle: ", robot.getTurret().getTargetAngle());
        telemetry.addData("Servo Pos: ", robot.getTurret().getServoPosition());
        telemetry.addData("TA: ", robot.getTurret().getTa());

        Scheduler.execute();

        // 12000deg/s (1666rpm) muito fraco
        // 25000deg/s bateu no TETO

        // 8500 config (3100rpm REAL) -> acertou por pouco
        // 8600 config (2950rpm REAL) -> correto!!

        // 8600 * 0.346 -> ~3000.
        // 8380 * 0.346 -> ~2900.

        // INTAKE VELOCITY IS SUPPOSED TO BE [8600].
        // MOTOR REPORTS [3000]RPM, NOT COUNTING FOR (5:1) REDUCTION.
        // UTILIZE THIS VALUE FOR PERFORMANCE.


        robot.getIntake().setThrottle(gamepad1.left_stick_y);

        if (gamepad1.dpadDownWasPressed()) {
            strength -= 0.1;
        } else if (gamepad1.dpadUpWasPressed()) {
            strength += 0.1;
        }

        telemetry.addData("Strength: ", strength);

        if (Math.abs(strength) > 0) {
            robot.getShooter().setTargetVelocity(RadiansPerSecond.of(1100 * strength * -gamepad1.left_trigger));
            robot.getShooter().prepareLaunch(gamepad1.left_trigger).schedule();
        } else if (Math.abs(strength) == 0) {
            robot.getShooter().setThrottle(-0.02);
            robot.getShooter().fullBrake().schedule();
        }

        if (gamepad1.right_bumper) {
            Command c = robot.getShooter().enableLaunch();
            Scheduler.schedule(c);
            telemetry.addData("SHOOTING: ", c.isScheduled());
        } else {
            robot.getShooter().disableLaunch().schedule();
        }

        telemetry.update();
    }
}
