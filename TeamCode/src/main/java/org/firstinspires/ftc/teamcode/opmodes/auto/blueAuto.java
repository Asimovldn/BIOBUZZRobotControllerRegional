package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.storage.Storage;

import com.pedropathing.paths.Path;

@Autonomous
public class blueAuto extends OpMode {
    private Follower follower;
    public static final PoseFactory p = PoseFactory.degrees();

    public static final Pose startPose = p.of(24, 24, 0);
    public static final Pose park = p.of(76, 80, 90);

    public static final Pose controlPose = p.of(36, 60, 45);

    @Override
    public void init() {
       follower = Constants.create(hardwareMap);
       follower.setPose(startPose);
    }

    @Override
    public void start() {
        follower.follow(goToShoot());
    }

    public static Path goToShoot() {
        return line(startPose, park).linear(startPose, park);
    }

    public static Path goBack() {
        return curve(park, controlPose, startPose).linear(park, startPose);
    }

    @Override
    public void loop() {
        follower.update();
    }

    @Override
    public void stop() {
        Storage.autonomousEnd = follower.pose();
    }
}
