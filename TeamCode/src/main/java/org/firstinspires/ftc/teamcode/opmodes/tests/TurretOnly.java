package org.firstinspires.ftc.teamcode.opmodes.tests;

import static dev.nextftc.units.Units.RotationsPerMinute;

import org.firstinspires.ftc.teamcode.storage.Storage;
import org.firstinspires.ftc.teamcode.systems.Bot;
import org.firstinspires.ftc.teamcode.systems.hardware.Turret;

import java.util.Arrays;

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
    public void start() {
        robot.getTurret().setMode(Turret.Mode.AIMING);
        robot.getTurret().reset();
    }

    @Override
    public void periodic() {
        telemetry.addData("Current Angle: ", robot.getTurret().getAngle());
        telemetry.addData("Current Err: ", robot.getTurret().getError());
        telemetry.addData("Current Speed: ", robot.getTurret().getSpeed());
        telemetry.addData("Target Angle: ", robot.getTurret().getTargetAngle());
        telemetry.addData("Servo Pos: ", robot.getTurret().getServoPosition());
        telemetry.addData("Location: ", robot.getTurret().currentLocation);
        telemetry.addData("Turret mode ", robot.getTurret().getMode().toString());
        telemetry.addData("Reading Tag: ", Arrays.toString(robot.getTurret().readingTag));
        telemetry.addData("LIMELIGHT QUEBROU PORRA: ", robot.getTurret().isLimelighteando);


        if (gamepad1.dpad_left) {
            ang--;
        } else if (gamepad1.dpad_right) {
            ang++;
        }

        telemetry.update();
    }
}
