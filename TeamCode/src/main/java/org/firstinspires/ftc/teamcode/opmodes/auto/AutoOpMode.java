package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Context;
import org.firstinspires.ftc.teamcode.RobotOpMode;
import org.firstinspires.ftc.teamcode.systems.hardware.Intake;
import org.firstinspires.ftc.teamcode.systems.hardware.Shooter;
import org.firstinspires.ftc.teamcode.systems.hardware.Turret;
import org.firstinspires.ftc.teamcode.systems.hardware.core.Mechanism;
import org.firstinspires.ftc.teamcode.util.LazyDrive;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

@Autonomous
public class AutoOpMode extends RobotOpMode {
    private static final Logger log = LoggerFactory.getLogger(AutoOpMode.class);
    PathBuilder paths;

    @Override
    public void init() {
        if (Context.runtime == null) {
            runtime = Context.setRuntime(new Context());
        } else {
            runtime = Context.runtime;
        }

        runtime.hardwareMap = hardwareMap;
        runtime.location = locationForOpMode();

        setAlliance(allianceForOpMode());

        intake = new Intake();
        shooter = new Shooter();
        turret = new Turret(runtime);

        drivetrain = new LazyDrive();

        follower = getFollower();

        for (Mechanism mech : getMechanisms()) {
            mech.passInRuntime(runtime);
            mech.setup();
        }

        paths = new PathBuilder();
        paths.mirrorPose(runtime);
        paths.build();

        create();
    }

    @Override
    public void start() {
        Scheduler.reset();

        setup();
        follower.update();
    }

    @Override
    public void loop() {
        follower = getFollower();

        updateMechanisms();
        periodic();

        Scheduler.execute();

        runtime.update(getAlliance(), null, follower.pose(), follower);
    }

    public static class PathBuilder {
        private PoseFactory poseFactory = PoseFactory.degrees();
        public PoseFactory getFactory() {
            return poseFactory;
        }

        public void mirrorPose(Context ctx) {
            if (ctx.GlobalAlliance == Context.Alliance.BLUE) {
                poseFactory = poseFactory.mirrorX(72.0);
                poseFactory = poseFactory.mirrorY(72.0);
            }
        }

        // Make Poses Below

        public void build() {
            startPoseA = poseFactory.of(83, 8.65, 90);
            parkPoseA = poseFactory.of(133, 30, 180);
        }

        public Pose startPoseA;
        public Pose parkPoseA;

        // Insert Paths Below

    }
}
