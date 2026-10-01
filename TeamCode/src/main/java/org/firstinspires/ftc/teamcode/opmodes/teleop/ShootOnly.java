package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static dev.nextftc.units.Units.DegreesPerSecond;
import static dev.nextftc.units.Units.RotationsPerMinute;

import android.view.animation.RotateAnimation;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.systems.Bot;

import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;

@NextTeleop
public class ShootOnly extends NextOpMode {
    public Bot robot;
    public ShootOnly(Bot bot) {
        super(bot);
        robot = bot;
    }

    @Override
    public void periodic() {
        telemetry.addData("Current Gamepad: ", gamepad1.left_stick_y);
        telemetry.addData("Current Velocity: ", robot.getShooter().Speed());

        robot.getShooter().setTargetVelocity(RotationsPerMinute.of(12000 * gamepad1.left_stick_y));

        telemetry.update();
    }
}
