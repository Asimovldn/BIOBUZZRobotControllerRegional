package org.firstinspires.ftc.teamcode.opmodes.tests;

import static dev.nextftc.units.Units.RadiansPerSecond;
import static dev.nextftc.units.Units.RotationsPerMinute;

import org.firstinspires.ftc.teamcode.systems.Bot;
import org.firstinspires.ftc.teamcode.systems.hardware.Turret;

import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;

@NextTeleop
public class TopDrums extends NextOpMode {
    public Bot robot;
    public TopDrums(Bot bot) {
        super(bot);
        robot = bot;
    }

    @Override
    public void start() {
        robot.getTurret().setMode(Turret.Mode.SEMI_AIM);
    }

    @Override
    public void periodic() {
        telemetry.addData("Current Gamepad: ", gamepad1.left_stick_y);
        telemetry.addData("Current Velocity: ", robot.getIntake().Speed().into(RotationsPerMinute));

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

        if (gamepad1.right_bumper) {
            robot.getTransfer().setThrottle(1);
        } else {
            robot.getTransfer().setThrottle(gamepad1.left_stick_y * 0.2);
        }

        double strength = 5.3 * -gamepad1.left_trigger;

        if (Math.abs(strength) > 2) {
            robot.getShooter().setTargetVelocity(RadiansPerSecond.of(1390 * strength));
        } else if (Math.abs(strength) == 0) {
            robot.getShooter().setThrottle(-0.02);
        }

        telemetry.addData("Shooter Speed: ", robot.getShooter().Speed());

        telemetry.update();
    }
}
