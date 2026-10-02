package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.pedropathing.ivy.Scheduler.execute;
import static com.pedropathing.ivy.Scheduler.schedule;

import static java.lang.Math.toDegrees;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.opmodes.auto.commands.Execute;
import org.firstinspires.ftc.teamcode.opmodes.auto.commands.PathBuilder;
import org.firstinspires.ftc.teamcode.opmodes.auto.commands.Routine;
import org.firstinspires.ftc.teamcode.storage.Storage;
import org.firstinspires.ftc.teamcode.systems.Bot;

import dev.nextftc.robot.opmode.NextAutonomous;
import dev.nextftc.robot.opmode.NextOpMode;

@NextAutonomous
public class NextBlueAuto extends NextOpMode {
    private final Bot robot;
    private PathBuilder paths;
    private Execute exec;
    private Routine routine;

    public MultipleTelemetry tele;

    public NextBlueAuto(Bot robot) {
        super(robot);
        this.robot = robot;
        Scheduler.reset();

        robot.setAlliance(Storage.Alliance.BLUE);

        paths = new PathBuilder();

        exec = new Execute(robot.getFollower(), robot);

        routine = new Routine(robot, paths, exec);

        tele = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
    }

    @Override
    public void start() {
        robot.getFollower().setPose(paths.startPoseA);
        schedule(routine.leaveAuto(Storage.Alliance.BLUE));
    }

    @Override
    public void periodic() {
        robot.getFollower().update();
        tele.update();

        Pose pose = robot.getFollower().pose();

        tele.addData("X: ", pose.x());
        tele.addData("Y: ", pose.y());
        tele.addData("OMEGA: ", toDegrees(pose.heading()));

        Scheduler.execute();
    }

    @Override
    public void end() {
        Storage.autonomousEnd = robot.getFollower().pose();
    }
}
