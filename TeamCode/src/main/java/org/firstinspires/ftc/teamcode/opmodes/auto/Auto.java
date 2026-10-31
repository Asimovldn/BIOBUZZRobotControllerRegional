package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.commands.Commands.waitUntil;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Context;
import org.firstinspires.ftc.teamcode.RobotOpMode;
import org.firstinspires.ftc.teamcode.systems.hardware.Hood;
import org.firstinspires.ftc.teamcode.systems.hardware.Intake;
import org.firstinspires.ftc.teamcode.systems.hardware.Shooter;
import org.firstinspires.ftc.teamcode.systems.hardware.Turret;
import org.firstinspires.ftc.teamcode.systems.hardware.core.Mechanism;
import org.firstinspires.ftc.teamcode.util.LazyDrive;
import org.firstinspires.ftc.teamcode.util.Util;
import org.jetbrains.annotations.Nullable;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.HashMap;

@Autonomous
public class Auto extends RobotOpMode {
    private static final Logger log = LoggerFactory.getLogger(Auto.class);
    Operator paths;

    /** Reduces the Auto to a single Code, for Runtime Owner Operations. */
    public String code() {
        return "BASECLASS";
    }

    @Override
    public void init() {
        if (Context.runtime == null) {
            runtime = Context.fromOwner(new Util.Code("AUTO", allianceForOpMode().toString(), code()), hardwareMap);
        } else {
            runtime = Context.runtime;
        }

        runtime.hardwareMap = hardwareMap;
        runtime.location = locationForOpMode();

        setAlliance(allianceForOpMode());

        intake = new Intake();
        shooter = new Shooter();
        turret = new Turret(runtime);
        hood = new Hood();

        drivetrain = new LazyDrive();

        follower = getFollower();

        for (Mechanism mech : getMechanisms()) {
            mech.passInRuntime(runtime);
            mech.setup();
        }

        paths = new Operator();
        paths.mirrorPose(runtime);
        paths.build();

        create();
    }

    @Override
    public void start() {
        Scheduler.reset();

        setup();

        if (routine() != null) {
            Scheduler.schedule(routine());
        }

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

    @Nullable
    public Command routine() {
        return null;
    }

    public static class Operator {
        private PoseFactory poseFactory = PoseFactory.degrees();
        public PoseFactory getFactory() {
            return poseFactory;
        }
        private boolean builded = false;

        public void mirrorPose(Context ctx) {
            if (ctx.GlobalAlliance == Context.Alliance.BLUE) {
                poseFactory = poseFactory.mirrorX(72.0);
                poseFactory = poseFactory.mirrorY(72.0);
            }
        }

        // Make Poses Below

        public void build() {
            getter.clear();

            startPoseA = poseFactory.of(59, 8, 90);
            gardenPose = poseFactory.of(9, 7, 180);
            parkPoseA = poseFactory.of(133, 30, 180);
            scoringRed = poseFactory.of(45, 120, 270);

            register("toGarden", line(startPoseA, gardenPose).facingPoint(gardenPose));
            register("toScoringRed", line(gardenPose, scoringRed).linear(gardenPose, scoringRed));

            builded = true;
        }

        public boolean ready() {
            return builded;
        }

        public static void register(String name, Path path) {
            if (getter.get(name) == null) {
                getter.replace(name, path);
                return;
            }

            getter.put(name, path);
        }

        public Command taskWait() {
            return waitUntil(this::ready);
        }

        public static Path get(String name) {
            return getter.get(name);
        }

        private static final HashMap<String, Path> getter = new HashMap<>();

        public Pose startPoseA;
        public Pose parkPoseA;
        public Pose gardenPose, scoringRed;
    }

    @Override
    public void stop() {
        Context.autonomousEnd = follower.pose();
        runtime.follower = null;
    }
}
