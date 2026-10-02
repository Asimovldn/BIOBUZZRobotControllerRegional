package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.Bot;

import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;

@NextTeleop
public class WalkOnly extends NextOpMode {
    public Bot robot;
    private Follower follower;
    public WalkOnly(Bot bot) {
        super(bot);
        robot = bot;
    }

    @Override
    public void start() {
        follower = robot.getFollower();
    }

    @Override
    public void periodic() {
        double speedDamp = 1 - gamepad1.left_trigger;
        double turn = -gamepad1.right_stick_x;

        follower.manual(
                -gamepad1.left_stick_y * speedDamp,
                gamepad1.left_stick_x * speedDamp,
                turn * speedDamp
        );

        follower.update();
    }
}
