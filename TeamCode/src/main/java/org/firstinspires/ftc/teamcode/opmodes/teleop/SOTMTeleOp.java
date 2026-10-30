package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Context;
import org.firstinspires.ftc.teamcode.RobotOpMode;
import org.firstinspires.ftc.teamcode.pedro.Drawing;
import org.firstinspires.ftc.teamcode.systems.hardware.Turret;
import org.firstinspires.ftc.teamcode.systems.sotm.SOTM;
import org.firstinspires.ftc.teamcode.systems.sotm.ShotData;
import org.firstinspires.ftc.teamcode.systems.sotm.ShotSolver;
import org.firstinspires.ftc.teamcode.util.Util;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;

@TeleOp(name = "SOTM Red", group = "TeleOp")
public class SOTMTeleOp extends RobotOpMode {
    private static final double READY_TURRET_DEG = 1.5;

    private static final Pose resetPoseA = new Pose(10, 132, Math.toRadians(270));
    private static final Pose resetPoseB = new Pose(59, 8, Math.toRadians(90));
    private static final Pose defaultStart = new Pose(59, 8, Math.toRadians(90));

    protected final SOTM sotm = new SOTM();
    protected boolean manualOnly = false;

    protected double currAngle = 20;

    private ShotSolver.Solution shot;
    private boolean sotmEnabled = true, spinEnabled = false;
    private double manualVelocity = 1300;
    private String lastLog = "";

    /** Only used when the runtime has no pose yet (OpMode started without an Auto before it). */
    public Pose startPose() {
        return forAlliance(defaultStart);
    }

    /** Red poses -> blue: the field is rotationally symmetric (same as Auto.mirrorPose). */
    private Pose forAlliance(Pose red) {
        if (getAlliance() != Context.Alliance.BLUE) return red;

        return new Pose(2 * Context.FIELD_CENTER - red.x(), 2 * Context.FIELD_CENTER - red.y(), Util.wrapRad(red.heading() + Math.PI));
    }

    @Override
    public void create() {
        if (runtime.pose == null) getFollower().setPose(startPose());

        if (runtime.owner == null) {
            runtime.setOwner(new Util.Code("SOTM", allianceForOpMode().toString()));
        };

        fill();
    }

    public void fill() {
        ShotData.load(sotm.table);
    }

    @Override
    public void setup() {
        Scheduler.reset();
        sotm.reset();
        getTurret().setMode(Turret.Mode.TRACK);

        Drawing.init();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void periodic() {
        Pose pose = follower.pose();
        Velocity vel = follower.velocity();
        sotm.update(pose, vel);


        Pose goal = goal();
        if (goal != null) getTurret().setRangeHint(Math.hypot(goal.x() - pose.x(), goal.y() - pose.y()));

        Double visionAngle = getTurret().consumeVisionAngle();
        if (visionAngle != null && goal != null && sotmEnabled) sotm.learnBias(visionAngle, pose, goal);

        shot = goal == null ? null : sotm.solve(pose, goal);
        boolean hasShot = shot != null && shot.valid;

        if (gamepad2.aWasPressed()) {
            sotmEnabled = !sotmEnabled;
            getTurret().setMode(sotmEnabled ? Turret.Mode.TRACK : Turret.Mode.AIMING);
            gamepad2.rumbleBlips(sotmEnabled ? 1 : 2);
        }

        if (sotmEnabled && hasShot) {
            getTurret().setTrackAngle(sotm.turretAngleDeg(shot));
        }

        if (gamepad2.dpadUpWasPressed()) manualVelocity += 10;
        if (gamepad2.dpadDownWasPressed()) manualVelocity -= 10;

        if (gamepad2.dpadLeftWasPressed()) currAngle--;
        if (gamepad2.dpadRightWasPressed()) currAngle++;

        if (gamepad2.leftTriggerWasPressed()) spinEnabled = !spinEnabled;
        if (gamepad2.leftTriggerWasReleased()) spinEnabled = !spinEnabled;
        if (gamepad2.leftBumperWasPressed()) manualOnly = !manualOnly;

        if (gamepad1.aWasPressed()) getFollower().setPose(forAlliance(resetPoseB));
        if (gamepad1.bWasPressed()) getFollower().setPose(forAlliance(resetPoseA));

        hood.setServoToPoses(hood.angleToServo(!sotm.table.isEmpty() && hasShot && !manualOnly ? shot.hood : currAngle));

        double velocity = !sotm.table.isEmpty() && hasShot && !manualOnly ? shot.velocity : manualVelocity;
        getShooter().setTargetVelocity(spinEnabled ? -velocity : 0);

        getIntake().setThrottle(gamepad2.left_stick_y);

        boolean turretReady = !sotmEnabled || getTurret().isOnTarget(READY_TURRET_DEG);
        boolean ready = getShooter().isReady() && turretReady;

        if (gamepad2.right_bumper && ready) {
            getShooter().enableLaunch().execute();
        } else {
            getShooter().disableLaunch().execute();
        }

        if (ready) gamepad2.rumble(100);

        if (gamepad2.yWasPressed()) {
            getTurret().reset();
            sotm.reset();
            sotm.resetBias();
            gamepad2.rumbleBlips(2);
        }

        if (gamepad2.xWasPressed()) {
            logShot();
            gamepad2.rumbleBlips(1);
        }

        if (gamepad2.bWasPressed()) {
            runtime.switchLocation();
            gamepad2.rumbleBlips(3);
        }

        drive(gamepad1);

        telemetry.addData("SOTM", sotmEnabled);
        telemetry.addData("Table", sotm.table.isEmpty() ? "EMPTY (fallback ToF, manual velocity)" : sotm.table.size() + " rows");
        telemetry.addData("Location", runtime.location);
        telemetry.addData("Pose", pose.toString());
        telemetry.addData("Speed (in/s)", sotm.speed());
        telemetry.addData("Omega (rad/s)", sotm.omega());
        telemetry.addData("Servo ", getTurret().getServoPosition());
        telemetry.addData("Runtime Owner", runtime.owner);

        if (hasShot) {
            telemetry.addData("Distance", shot.distance);
            telemetry.addData("ToF", shot.tof);
            telemetry.addData("Aim point", "%.1f, %.1f", shot.aimX, shot.aimY);
            telemetry.addData("Turret cmd", sotm.turretAngleDeg(shot));
        }

        telemetry.addData("Turret", getTurret().getAngle());
        telemetry.addData("Turret err", getTurret().getError());
        telemetry.addData("Flywheel", getShooter().Speed());
        telemetry.addData("Flywheel target", getShooter().getTargetVelocity());
        telemetry.addData("Ready", ready);
        telemetry.update();

        Drawing.drawDebug(follower);
    }

    private void logShot() {
        if (shot == null || !shot.valid) return;

        String line = String.format(Locale.US, "table.add(%.1f, %.0f, %.1f, %.2f);", shot.distance, manualVelocity, currAngle, shot.tof);

        try (FileWriter f = new FileWriter(AppUtil.getInstance().getSettingsFile("shot_table.txt"), true)) {
            f.write(line + "\n");
            lastLog = line;
        } catch (IOException e) {
            lastLog = "log failed: " + e.getMessage();
        }
    }

    private Pose goal() {
        try {
            return new Context.HiveLocations(runtime).get();
        } catch (Exception e) {
            telemetry.addData("hey check this out: ", e.toString());
            return null;
        }
    }
}
