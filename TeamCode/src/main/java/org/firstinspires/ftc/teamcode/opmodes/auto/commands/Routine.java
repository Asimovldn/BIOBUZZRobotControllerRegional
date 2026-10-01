package org.firstinspires.ftc.teamcode.opmodes.auto.commands;

import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.storage.Storage;
import org.firstinspires.ftc.teamcode.systems.Bot;

public class Routine {
    private Bot robot;
    private Follower follower;
    private PathBuilder paths;
    private Execute exec;

    public Routine(Bot robot, PathBuilder paths, Execute exec) {
        this.robot = robot;
        follower = robot.getFollower();
        this.paths = paths;
        this.exec = exec;
        paths.mirrorPose(robot.getAlliance());
    }

    public Command leaveAuto(Storage.Alliance alliance) {
        paths.mirrorPose(alliance);
        return sequential(
                exec.run(paths.startToLeave())
        );
    }
}
