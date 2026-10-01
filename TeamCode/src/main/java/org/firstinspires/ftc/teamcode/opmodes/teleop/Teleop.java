package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static dev.nextftc.units.Units.RotationsPerMinute;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Scheduler;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.storage.Storage;
import org.firstinspires.ftc.teamcode.systems.Bot;
import org.firstinspires.ftc.teamcode.systems.hardware.Turret;

import dev.nextftc.hardware.RobotController;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;

@NextTeleop
public class Teleop extends NextOpMode {
    private final Bot robot;
    // private final Follower follower;

    private final CommandGamepad driver;
    private final CommandGamepad tops;

    public Teleop(Bot bot) {
        super(bot);
        robot = bot;
        // follower = Constants.create(RobotController.hardwareMap());

        Scheduler.reset();

        driver = new CommandGamepad(gamepad1);
        tops = new CommandGamepad(gamepad2);
    }

    private Turret turret() {
        return robot.getTurret();
    }

    @Override
    public void periodic() {
        /*
        double speedDamp = 1 - gamepad1.left_trigger;
        double turn = -gamepad1.right_stick_x;

        follower.manual(
                -gamepad1.left_stick_y * speedDamp,
                -gamepad1.left_stick_x * speedDamp,
                turn * speedDamp
        );
        */

        // Bot Angular Velocity --> velocidade de rotação para compensar na torreta.
        // robot.getTurret().botAngularVelocity(follower.velocity().omega);


        // follower.update();

        if (robot.getAlliance() != Storage.GlobalAlliance) {
            robot.setAlliance(Storage.GlobalAlliance);
            robot.getTurret().setAlliance(Storage.GlobalAlliance);
        }

        tops.leftBumper()
                .onTrue(turret().changeLocation());

        telemetry.addData("Turret Error: ", robot.getTurret().getError());
        telemetry.addData("Turret Speed: ", robot.getTurret().getSpeed());
        telemetry.addData("Degrees: ", robot.getTurret().degs);
        telemetry.addData("Called Periodic: ", robot.getTurret().calledPeriodic);

        robot.getShooter().setTargetVelocity(RotationsPerMinute.of(4000));

        telemetry.addData("Velocidade Atual: ", robot.getShooter().Speed().into(RotationsPerMinute));

        telemetry.update();
    };
}
