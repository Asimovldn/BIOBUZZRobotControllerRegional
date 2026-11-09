package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import static org.firstinspires.ftc.teamcode.util.Util.compare;

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
import org.firstinspires.ftc.teamcode.systems.hardware.Ramp;
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
        public static double flowerRampMs = 120;
        public static double feedMs = 900;
        public static double shootMs = 2100;
        public static double delayShootMs = 85;
        public static double pathTimeoutMs = 2500;
        public static double flowerTimeoutMs = 600;
        public static double longPathTimeoutMs = 3500;
        public static double pathToleranceIn = 1.5;
        public static double handoffToleranceIn = 6;
        public static double pathToleranceDeg = 10;
        public static double finalExtraFeedMs = 1200;
        public static double gardenDwellMs = 300;
        public static double turretReadyDeg = 1.25;
        public static boolean earlyFinish = true;
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
        ramp = new Ramp();

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

    /** Points turret and ramp at the current goal and, when {@code autoSpin} is on, sets the flywheel speed. */
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

    protected Command manual(double f, double s, double t) {
        return instant(() -> follower.manual(f, s, t));
    }

    protected Command parkRoutine() {
        return sequential(
                followPath("ParkPose")
        );
    }

    /** Audience starts from the start pose. Scoring assumes the robot is already pressed against the scoring flower. */
    protected Command flowerRoutine(String where) {
        if (compare(where, "Audience")) {
            return sequential(
                    setIntake(0),
                    instant(() -> ramp.set(ramp.on())),
                    followPath("preFlowerAudience"),
                    followPath("flowerAudience", AutoTune.flowerTimeoutMs),
                    setIntake(1),
                    waitMs(AutoTune.feedMs * 0.8),
                    setIntake(0.5),
                    followPath("backupAudience"),
                    waitMs(AutoTune.flowerRampMs),
                    setIntake(1),
                    followPath("flowerAudience", AutoTune.flowerTimeoutMs, AutoTune.handoffToleranceIn)
            );
        }

        return sequential(
                instant(() -> ramp.set(ramp.on())),
                waitMs(AutoTune.flowerRampMs / 3),
                setIntake(1),
                followPath("flowerScoring", AutoTune.flowerTimeoutMs),
                waitMs(AutoTune.flowerRampMs),
                setIntake(0.5),
                followPath("backupScoring", AutoTune.flowerTimeoutMs),
                waitMs(AutoTune.flowerRampMs),
                setIntake(1),
                followPath("flowerScoring", AutoTune.flowerTimeoutMs, AutoTune.handoffToleranceIn),
                setIntake(1)
        );
    }

    protected Command scoringRoutineWithFlower() {
        return sequential(
                setIntake(0.5),
                followPath("backupScoring", AutoTune.flowerTimeoutMs),
                waitMs(AutoTune.flowerRampMs),
                setIntake(1),
                followPath("flowerScoring", AutoTune.flowerTimeoutMs, AutoTune.handoffToleranceIn)
        );
    }

    protected Command gardenRoutine() {
        return sequential(
                instant(() -> ramp.set(ramp.off())),
                followPath("preGarden"),
                setIntake(1),
                instant(() -> ramp.set(ramp.off())),
                followPath("toGarden"),
                waitMs(AutoTune.gardenDwellMs * 0.8),
                instant(() -> ramp.set(ramp.on())),
                followPath("backupGarden"),
                setIntake(1),
                instant(() -> ramp.set(ramp.off())),
                followPath("toGarden")
        );
    }

    /** Waits for the flywheel and turret (with a timeout), then feeds for {@code shootMs}. */
    protected Command shoot() {
        return shoot(AutoTune.shootMs);
    }

    protected Command shoot(double ms) {
        return sequential(
                race(waitUntil(this::readyToShoot), waitMs(AutoTune.readyTimeoutMs)),
                waitMs(AutoTune.delayShootMs),
                feed(ms)
        );
    }

    protected Command feed() {
        return feed(AutoTune.shootMs);
    }

    protected Command feed(double ms) {
        final AtomicLong startNs = new AtomicLong();
        return infinite(() -> {
            shooter.setFeeding(readyToShoot());
            intake.setThrottle(1);
        })
                .setStart(() -> {
                    startNs.set(System.nanoTime());
                    intake.setThrottle(1);
                })
                .setDone(() -> (System.nanoTime() - startNs.get()) * 1e-6 >= ms)
                .setEnd(c -> {
                    shooter.setFeeding(false);
                    intake.setThrottle(0);
                });
    }

    protected Command flowerSwitch() {
        return Command.build()
                .setExecute(ramp::alter)
                .setDone(() -> true);
    }

    protected Command followPath(String name) {
        return followPath(name, AutoTune.pathTimeoutMs, AutoTune.pathToleranceIn);
    }

    protected Command followPath(String name, double timeout) {
        return followPath(name, timeout, AutoTune.pathToleranceIn);
    }

    /** Ends on timeout or as soon as the robot is within {@code toleranceIn} of the end pose, then holds it. */
    protected Command followPath(String name, double timeoutMs, double toleranceIn) {
        final Path path = Operator.get(name);

        return sequential(
                race(
                        follow(follower, path),
                        race(
                                waitMs(timeoutMs),
                                waitUntil(() -> AutoTune.earlyFinish && atPose(path.endPose(), toleranceIn))
                        )
                ),
                instant(() -> {
                    if (AutoTune.holdAfterPath) follower.hold(path.endPose());
                })
        );
    }

    private boolean atPose(Pose target, double toleranceIn) {
        Pose pose = follower.pose();
        double headingErr = Math.toDegrees(Math.abs(Util.wrapRad(pose.heading() - target.heading())));

        return Math.hypot(pose.x() - target.x(), pose.y() - target.y()) < toleranceIn
                && headingErr < AutoTune.pathToleranceDeg;
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

    protected Command setRamp(double pos) {
        return instant(() -> ramp.set(pos));
    }

    protected double on() {
        return ramp.on();
    }

    protected double off() {
        return ramp.off();
    }

    protected Command setRampMax() {
        return instant(() -> ramp.maxUp());
    }

    protected Command stopSpin() {
        return instant(() -> {
            autoSpin = false;
            shooter.brakeNow();
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
        telemetry.addData("flower servo: ", ramp.getPosition());
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
            gardenPose = poseFactory.of(12.7, 8.2, 180);
            preFlowerScoring = poseFactory.of(49.5, 115, 90);
            flowerScoringPose = poseFactory.of(49.1, 131.7, 90);

            preFlowerAudience = poseFactory.of(26, 49.5, 180);
            flowerAudiencePose = poseFactory.of(11.7, 49.5, 180);

            preGardenPose = poseFactory.of(26, 8.7, 180);
            parkPoseA = poseFactory.of(11.4, 114, 0);
            scoringPose = poseFactory.of(48.25, 127, 270);

            Pose flowerToFlowerCtrl = poseFactory.of(18.2, 99, 0);
            Pose gardenToScoringCtrl = poseFactory.of(17.45, 80.7, 0);
            Pose flowerToFlowerCtrl2 = poseFactory.of(46.5, 105, 0);

            register("toGarden", line(preGardenPose, gardenPose).linear(preGardenPose, gardenPose));
            register("preGarden", line(startPoseA, preGardenPose).tangent());

            register("fromFlowerToFlower", curve(flowerAudiencePose, flowerToFlowerCtrl, flowerToFlowerCtrl2, flowerScoringPose).linear(flowerAudiencePose, flowerScoringPose));

            register("toScoring", curve(flowerAudiencePose, flowerToFlowerCtrl, flowerToFlowerCtrl2, scoringPose).linear(flowerAudiencePose, scoringPose));
            register("fromGardenScore", curve(gardenPose, gardenToScoringCtrl, scoringPose).reverseTangent());

            register("flowerScoring", line(preFlowerScoring, flowerScoringPose).linear(preFlowerScoring, flowerScoringPose));

            register("preFlowerAudience", line(startPoseA, preFlowerAudience).linear(startPoseA, preFlowerAudience));
            register("flowerAudience", line(preFlowerAudience, flowerAudiencePose).linear(preFlowerAudience, flowerAudiencePose));

            register("backupAudience", line(flowerAudiencePose, preFlowerAudience).linear(flowerAudiencePose, preFlowerAudience));
            register("backupScoring", line(flowerScoringPose, preFlowerScoring).linear(flowerScoringPose, preFlowerScoring));
            register("backupGarden", line(gardenPose, preGardenPose).linear(gardenPose, preGardenPose));

            register("ParkPose", line(flowerScoringPose, parkPoseA).linear(flowerScoringPose, parkPoseA));

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
        public Pose gardenPose, scoringPose, preGardenPose, flowerScoringPose, preFlowerScoring;

        public Pose preFlowerAudience, flowerAudiencePose;
    }
}