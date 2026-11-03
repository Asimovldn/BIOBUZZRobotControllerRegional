package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Context;
import org.firstinspires.ftc.teamcode.RobotOpMode;
import org.firstinspires.ftc.teamcode.pedro.Drawing;
import org.firstinspires.ftc.teamcode.systems.hardware.Hood;
import org.firstinspires.ftc.teamcode.systems.hardware.Intake;
import org.firstinspires.ftc.teamcode.systems.hardware.Shooter;
import org.firstinspires.ftc.teamcode.systems.hardware.Turret;
import org.firstinspires.ftc.teamcode.systems.hardware.core.Mechanism;
import org.firstinspires.ftc.teamcode.systems.sotm.SOTM;
import org.firstinspires.ftc.teamcode.systems.sotm.ShotData;
import org.firstinspires.ftc.teamcode.systems.sotm.ShotSolver;
import org.firstinspires.ftc.teamcode.util.LazyDrive;
import org.firstinspires.ftc.teamcode.util.Util;
import org.jetbrains.annotations.Nullable;

import java.util.HashMap;
import java.util.concurrent.atomic.AtomicLong;

/**
 * Base class for autonomous routines. Not registered as an OpMode, subclasses carry {@code @Autonomous}.
 * Alliance comes from {@link #allianceForOpMode()}, poses are written for RED and mirrored for BLUE.
 */
public abstract class Auto extends RobotOpMode {

    @Config
    public static class AutoTune {
        public static double readyTimeoutMs = 2500;
        public static double feedMs = 2000;
        public static double pathTimeoutMs = 6000;
        public static double gardenDwellMs = 400;
        public static double turretReadyDeg = 2;
        public static boolean holdAfterPath = true;
        public static boolean keepSpinning = true;
        public static boolean learnBias = true;
    }

    protected Operator paths;
    protected final SOTM sotm = new SOTM();
    protected Context.HiveLocations locs;
    protected ShotSolver.Solution shot;
    protected boolean autoSpin = false;

    private Command plan;
    private final ElapsedTime matchTimer = new ElapsedTime();

    /** Reduces the Auto to a single Code, for Runtime Owner Operations. */
    public String code() {
        return "BASECLASS";
    }

    @Nullable
    public Command routine() {
        return null;
    }

    @Override
    public void init() {
        runtime = Context.fromOwner(new Util.Code("AUTO", allianceForOpMode().toString(), code()), hardwareMap);
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

        follower.setPose(paths.startPose());
        runtime.pose = paths.startPose();

        locs = new Context.HiveLocations(runtime);
        ShotData.load(sotm.table);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Drawing.init();

        create();
        plan = routine();
    }

    @Override
    public void init_loop() {
        follower.update();

        Pose pose = follower.pose();
        Pose expected = paths.startPose();

        telemetry.addData("Alliance", getAlliance());
        telemetry.addData("Routine", code());
        telemetry.addData("Pose", "%.1f, %.1f, %.0f deg", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Expected", "%.1f, %.1f, %.0f deg", expected.x(), expected.y(), Math.toDegrees(expected.heading()));
        telemetry.addData("Pose error (in)", Math.hypot(pose.x() - expected.x(), pose.y() - expected.y()));
        telemetry.addData("Paths", Operator.count());
        telemetry.addData("Shot table", sotm.table.size() + " rows");
        telemetry.addData("Routine built", plan != null);
        telemetry.update();
    }

    @Override
    public void start() {
        Scheduler.reset();
        matchTimer.reset();

        setup();

        getTurret().setMode(Turret.Mode.TRACK);

        if (plan != null) Scheduler.schedule(plan);

        follower.update();
    }

    @Override
    public void loop() {
        follower.update();

        aim();
        periodic();

        Scheduler.execute();
        updateMechanisms();

        runtime.update(getAlliance(), null, follower.pose(), follower);

        report();
    }

    @Override
    public void stop() {
        Scheduler.reset();

        if (shooter != null) shooter.brakeNow();
        if (intake != null) intake.setThrottle(0);

        if (follower != null) {
            Pose end = follower.pose();
            Context.autonomousEnd = end;
            if (runtime != null) runtime.pose = end;
        }

        if (runtime != null) runtime.follower = null;
    }

    /** Points turret and hood at the current goal and, when {@code autoSpin} is on, sets the flywheel speed. */
    protected void aim() {
        Pose goal = locs.get();
        if (goal == null) {
            shot = null;
            return;
        }

        Pose pose = follower.pose();
        sotm.update(pose, follower.velocity());

        // turret.setRangeHit(Math.hypot(goal.x() - pose.x(), goal.y() - pose.y()));

        Double vision = turret.consumeVisionAngle();
        if (vision != null && AutoTune.learnBias) sotm.learnBias(vision, pose, goal);

        ShotSolver.Solution solution = sotm.solve(pose, goal);
        shot = solution.valid ? solution : null;
        if (shot == null) return;

        turret.setTrackAngle(sotm.turretAngleDeg(shot));
        hood.setServoToPoses(hood.angleToServo(shot.hood));

        if (autoSpin) shooter.spinTo(shot.velocity);
    }

    protected boolean readyToShoot() {
        return shot != null
                && turret.getMode() == Turret.Mode.TRACK
                && turret.isOnTarget(AutoTune.turretReadyDeg)
                && shooter.isReady();
    }

    protected Command spin(boolean on) {
        return instant(() -> {
            autoSpin = on;
            if (!on) shooter.brakeNow();
        });
    }

    /** Waits for the flywheel and turret (with a timeout), then feeds for {@code feedMs}. */
    protected Command shoot() {
        return sequential(
                race(waitUntil(this::readyToShoot), waitMs(AutoTune.readyTimeoutMs)),
                feed()
        );
    }

    protected Command feed() {
        final AtomicLong startNs = new AtomicLong();

        return Command.build()
                .setStart(() -> {
                    startNs.set(System.nanoTime());
                    intake.setThrottle(1);
                })
                .setExecute(() -> shooter.setFeeding(readyToShoot()))
                .setDone(() -> (System.nanoTime() - startNs.get()) * 1e-6 >= AutoTune.feedMs)
                .setEnd(c -> {
                    shooter.setFeeding(false);
                    intake.setThrottle(0);
                });
    }

    /** Follows a registered path with a timeout, then holds the end pose. */
    protected Command followPath(String name) {
        final Path path = Operator.get(name);

        return sequential(
                race(follow(follower, path), waitMs(AutoTune.pathTimeoutMs)),
                instant(() -> {
                    if (AutoTune.holdAfterPath) follower.hold(path.endPose());
                })
        );
    }

    protected Command setIntake(double power) {
        return instant(() -> intake.setThrottle(power));
    }

    protected Command stopSpinUnlessKept() {
        return instant(() -> {
            if (!AutoTune.keepSpinning) {
                autoSpin = false;
                shooter.brakeNow();
            }
        });
    }

    private void report() {
        Pose pose = follower.pose();

        telemetry.addData("Time (s)", "%.1f", matchTimer.seconds());
        telemetry.addData("Pose", "%.1f, %.1f, %.0f deg", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Follower busy", follower.isBusy());
        telemetry.addData("Location", runtime.location);

        if (shot != null) {
            telemetry.addData("Goal distance", shot.distance);
            telemetry.addData("Turret err", turret.getError());
        }

        telemetry.addData("Flywheel", shooter.Speed());
        telemetry.addData("Flywheel target", shooter.getTargetVelocity());
        telemetry.addData("Ready", readyToShoot());
        telemetry.update();

        Drawing.drawDebug(follower);
    }

    public static class Operator {
        private PoseFactory poseFactory = PoseFactory.degrees();
        private boolean builded = false;

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
            getter.clear();

            startPoseA = poseFactory.of(59, 8, 90);
            gardenPose = poseFactory.of(9, 7, 180);
            parkPoseA = poseFactory.of(133, 30, 180);
            scoringPose = poseFactory.of(45, 120, 270);

            register("toGarden", line(startPoseA, gardenPose).linear(startPoseA, gardenPose));
            register("toScoring", line(gardenPose, scoringPose).linear(gardenPose, scoringPose));

            builded = true;
        }

        public Pose startPose() {
            return startPoseA;
        }

        public boolean ready() {
            return builded;
        }

        public static void register(String name, Path path) {
            getter.put(name, path);
        }

        public Command taskWait() {
            return waitUntil(this::ready);
        }

        public static Path get(String name) {
            Path path = getter.get(name);
            if (path == null) throw new IllegalStateException("Path not registered: " + name);
            return path;
        }

        public static int count() {
            return getter.size();
        }

        private static final HashMap<String, Path> getter = new HashMap<>();

        public Pose startPoseA;
        public Pose parkPoseA;
        public Pose gardenPose, scoringPose;
    }
}
