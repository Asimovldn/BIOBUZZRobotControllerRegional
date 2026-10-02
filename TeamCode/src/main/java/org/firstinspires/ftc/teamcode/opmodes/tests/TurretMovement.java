package org.firstinspires.ftc.teamcode.opmodes.tests;

import static org.firstinspires.ftc.teamcode.storage.Storage.isEqualTo;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.storage.Storage;
import org.firstinspires.ftc.teamcode.systems.Bot;
import org.firstinspires.ftc.teamcode.systems.hardware.Shooter;

import java.util.Optional;
import java.util.OptionalDouble;

import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextUtility;
import dev.nextftc.robot.triggers.CommandGamepad;

@Config
@NextUtility
public class TurretMovement extends NextOpMode {
    public Bot robot;

    public boolean POSITIONAL_TESTING;
    public Storage.Alliance alliance = Storage.Alliance.BLUE;
    public CommandGamepad gmp = new CommandGamepad(gamepad1);

    public TurretMovement(Bot bot) {
        super(bot);
        robot = bot;

    }

    @Override
    public void disabledPeriodic() {
        telemetry.addData("Current Alliance: ", alliance);
        if (gamepad1.leftBumperWasPressed()) {
            alliance = (Storage.Alliance) isEqualTo(alliance, Storage.Alliance.BLUE, Storage.Alliance.RED);
            telemetry.update();
        }
    }

    @Override
    public void periodic() {
    }
}
