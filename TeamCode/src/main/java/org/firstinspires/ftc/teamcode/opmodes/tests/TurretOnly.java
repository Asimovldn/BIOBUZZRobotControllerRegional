package org.firstinspires.ftc.teamcode.opmodes.tests;

import static dev.nextftc.units.Units.RotationsPerMinute;

import org.firstinspires.ftc.teamcode.storage.Storage;
import org.firstinspires.ftc.teamcode.systems.Bot;
import org.firstinspires.ftc.teamcode.systems.hardware.Turret;

import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;

@NextTeleop
public class TurretOnly extends NextOpMode {
    public Bot robot;
    private double ang = 0;
    public TurretOnly(Bot bot) {
        super(bot);
        robot = bot;

        robot.setAlliance(Storage.Alliance.RED);
        robot.getTurret().setAlliance(Storage.Alliance.RED);
    }

    @Override
    public void periodic() {
        telemetry.addData("Current Angle: ", robot.getTurret().getAngle());
        telemetry.addData("Current Err: ", robot.getTurret().getError());
        telemetry.addData("Current Speed: ", robot.getTurret().getSpeed());

        // 12000deg/s (1666rpm) muito fraco
        // 25000deg/s bateu no TETO

        // 8500 config (3100rpm REAL) -> acertou por pouco
        // 8600 config (2950rpm REAL) -> correto!!

        // 8600 * 0.346 -> ~3000.
        // 8380 * 0.346 -> ~2900.

        // INTAKE VELOCITY IS SUPPOSED TO BE [8600].
        // MOTOR REPORTS [3000]RPM, NOT COUNTING FOR (5:1) REDUCTION.
        // UTILIZE THIS VALUE FOR PERFORMANCE.


        robot.getShooter().setTargetVelocity(RotationsPerMinute.of(8380 * gamepad1.left_stick_y));

        if (gamepad1.dpad_left) {
            ang--;
        } else if (gamepad1.dpad_right) {
            ang++;
        }

        robot.getTurret().setManualAngle(ang);

        telemetry.update();
    }
}
