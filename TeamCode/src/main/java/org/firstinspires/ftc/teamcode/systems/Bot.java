package org.firstinspires.ftc.teamcode.systems;

import androidx.annotation.NonNull;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.storage.Storage;
import org.firstinspires.ftc.teamcode.systems.hardware.Intake;
import org.firstinspires.ftc.teamcode.systems.hardware.Shooter;
import org.firstinspires.ftc.teamcode.systems.hardware.Transfer;
import org.firstinspires.ftc.teamcode.systems.hardware.Turret;
import org.jetbrains.annotations.NonNls;

import java.util.Set;

import dev.nextftc.hardware.RobotController;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.NextRobot;
import dev.nextftc.robot.drive.DriveCommands;

public class Bot implements NextRobot {
    private Follower follower;
    private Turret turret;
    private Shooter shooter;
    private Transfer transfer;
    private Intake intake;
    private Storage.Alliance alliance;
    private Storage constants;

    public Bot() {
        constants = new Storage();
        intake = new Intake();
        transfer = new Transfer();
        shooter = new Shooter();
        // turret = new Turret();
    };


    public Turret getTurret() {
        return turret;
    }

    public Shooter getShooter() { return shooter; }

    public Intake getIntake() { return intake; }

    public Transfer getTransfer() { return transfer; }

    public Follower getFollower() {
        if (follower == null) {
            follower = Constants.create(RobotController.hardwareMap());
        }

        return follower;
    }

    public void setAlliance(Storage.Alliance alliance) {
        this.alliance = alliance;
        constants.editSetting("GlobalAlliance", alliance);
    }

    public Storage.Alliance getAlliance() {
        return alliance;
    }

    public void startDrive(Gamepad gmp) {
        ManualDrive.driveOrHold(getFollower(), -gmp.left_stick_y, -gmp.left_stick_x, -gmp.right_stick_x);
    }

    @NonNull
    @Override
    public Set<Mechanism> getMechanisms() {
        return Set.of(intake, transfer, shooter);
    }
}
