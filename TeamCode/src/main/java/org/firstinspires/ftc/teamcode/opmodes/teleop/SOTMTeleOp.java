package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static com.pedropathing.api.Paths.line;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Context;
import org.firstinspires.ftc.teamcode.RobotOpMode;
import org.firstinspires.ftc.teamcode.pedro.Drawing;
import org.firstinspires.ftc.teamcode.systems.hardware.Turret;
import org.firstinspires.ftc.teamcode.systems.sotm.SOTM;
import org.firstinspires.ftc.teamcode.systems.sotm.ShotData;
import org.firstinspires.ftc.teamcode.systems.sotm.ShotSolver;
import org.firstinspires.ftc.teamcode.systems.sotm.ShotTable;
import org.firstinspires.ftc.teamcode.util.ResetPoses;
import org.firstinspires.ftc.teamcode.util.Util;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;

@TeleOp(name = "SOTM Red", group = "TeleOp")
public class SOTMTeleOp extends RobotOpMode {
    private static final double READY_TURRET_DEG = 4;

    private static final Pose defaultStart = ResetPoses.AUDIENCE_START;

    protected final SOTM sotm = new SOTM();
    protected boolean manualOnly = false;

    private ShotSolver.Solution shot;
    private boolean sotmEnabled = true, spinEnabled = false;

    private double manualVelocity = 1300;
    private String lastLog = "";

    /** Only used when the runtime has no pose yet (OpMode started without an Auto before it). */
    public Pose startPose() {
        return forAlliance(defaultStart);
    }

    /** Red poses -> blue: the field is rotationally symmetric (same as Auto.mirrorPose). */
    protected Pose forAlliance(Pose red) {
        return ResetPoses.forAlliance(red, getAlliance() == Context.Alliance.BLUE);
    }

    protected void resetPose(Pose pose) {
        getFollower().setPose(pose);
        rumblePad().rumbleBlips(1);
    }

    protected static class Controls {
        boolean spin, feed, toggleSotm, velUp, velDown, toggleManual, resetTurret, logShot, rampAlter, rampMax;
        double intake;
    }

    protected Controls controls() {
        Controls c = new Controls();
        c.toggleSotm = gamepad2.aWasPressed();
        c.velUp = gamepad2.dpadUpWasPressed();
        c.velDown = gamepad2.dpadDownWasPressed();
        c.spin = gamepad2.left_trigger > 0.5;
        c.toggleManual = gamepad2.leftBumperWasPressed();
        c.rampAlter = gamepad1.rightStickButtonWasPressed();
        c.rampMax = gamepad1.leftStickButtonWasPressed();
        c.intake = gamepad2.left_stick_y;
        c.feed = gamepad2.right_bumper;
        c.resetTurret = gamepad2.yWasPressed();
        c.logShot = gamepad2.xWasPressed();
        return c;
    }

    protected Gamepad rumblePad() {
        return gamepad2;
    }

    protected void handleResets() {
        Pose now = getFollower().pose();

        if (gamepad1.aWasPressed()) resetPose(forAlliance(ResetPoses.AUDIENCE_START));
        if (gamepad1.bWasPressed()) resetPose(forAlliance(ResetPoses.SCORING_CORNER));
        if (gamepad1.xWasPressed()) resetPose(forAlliance(ResetPoses.GARDEN));
        if (gamepad1.yWasPressed()) resetPose(forAlliance(ResetPoses.SCORING_FLOWER));

        boolean front = gamepad1.left_bumper;
        if (gamepad1.dpadDownWasPressed()) resetPose(wallPose(now, ResetPoses.Wall.AUDIENCE, front));
        if (gamepad1.dpadUpWasPressed()) resetPose(wallPose(now, ResetPoses.Wall.SCORING, front));
        if (gamepad1.dpadLeftWasPressed()) resetPose(wallPose(now, ResetPoses.Wall.LEFT, front));
        if (gamepad1.dpadRightWasPressed()) resetPose(wallPose(now, ResetPoses.Wall.RIGHT, front));
    }

    protected Pose wallPose(Pose now, ResetPoses.Wall wall, boolean front) {
        return front ? ResetPoses.wallFront(now, wall) : ResetPoses.wall(now, wall);
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
        Controls c = controls();

        Pose goal = goal();
        // if (goal != null) getTurret().setRangeHint(Math.hypot(goal.x() - pose.x(), goal.y() - pose.y()));

        Double visionAngle = getTurret().consumeVisionAngle();
        if (visionAngle != null && goal != null && sotmEnabled) sotm.learnBias(visionAngle, pose, goal, runtime.location);

        shot = goal == null ? null : sotm.solve(pose, goal, runtime.location);
        boolean hasShot = shot != null && shot.valid;

        if (c.toggleSotm) {
            sotmEnabled = !sotmEnabled;
            getTurret().setMode(sotmEnabled ? Turret.Mode.TRACK : Turret.Mode.AIMING);
            rumblePad().rumbleBlips(sotmEnabled ? 1 : 2);
        }

        if (sotmEnabled && hasShot) {
            getTurret().setTrackAngle(sotm.turretAngleDeg(shot));
        }

        if (c.velUp) manualVelocity += 10;
        if (c.velDown) manualVelocity -= 10;

        spinEnabled = c.spin;
        if (c.toggleManual) manualOnly = !manualOnly;

        handleResets();

        if (gamepad1.rightStickButtonWasPressed()) {
            ramp.alter();
        }

        if (gamepad1.leftStickButtonWasPressed()) {
            ramp.maxUp();
        }

        double velocity = !sotm.table.isEmpty() && hasShot && !manualOnly ? shot.velocity : manualVelocity;
        getShooter().spinTo(spinEnabled ? velocity : 0);

        getIntake().setThrottle(c.intake);

        boolean turretReady = !sotmEnabled || getTurret().isOnTarget(READY_TURRET_DEG);
        boolean ready = getShooter().isReady() && turretReady;

        getShooter().setFeeding(c.feed && getShooter().canFeed() && turretReady);

        if (ready) rumblePad().rumble(100);

        if (c.resetTurret) {
            getTurret().reset();
            sotm.reset();
            sotm.resetBias();
            rumblePad().rumbleBlips(2);
        }


        if (c.logShot) {
            logShot();
            rumblePad().rumbleBlips(1);
        }

        drive(gamepad1);

        telemetry.addData("SOTM", sotmEnabled);
        telemetry.addData("Table", sotm.table.isEmpty() ? "EMPTY (fallback ToF, manual velocity)" : sotm.table.size() + " rows");
        telemetry.addData("Location", runtime.location);
        telemetry.addData("Pose", pose.toString());
        speedCheck(pose, vel);
        telemetry.addData("Speed (in/s)", sotm.speed());
        telemetry.addData("Omega (rad/s)", sotm.omega());
        telemetry.addData("Servo ", getTurret().getServoPosition());
        telemetry.addData("Runtime Owner", runtime.owner);
        telemetry.addData("Flower Angle: ", ramp.servo.getPosition());
        telemetry.addData("Flower Get Flag: ", ramp.getFlag());

        if (hasShot) {
            telemetry.addData("Distance", shot.distance);
            telemetry.addData("ToF", shot.tof);
            telemetry.addData("ToF (ballistic est.)", ShotTable.ballisticTof(shot.distance));
            telemetry.addData("Aim point", "%.1f, %.1f", shot.aimX, shot.aimY);
            telemetry.addData("Lead (in)", "%.1f (gain %.2f, cap %.0f)", shot.leadIn, ShotSolver.SolverTune.velocityGain, ShotSolver.SolverTune.maxLeadIn);
            telemetry.addData("Latency shift (in)", shot.latencyIn);
            telemetry.addData("Turret lead (deg)", getTurret().getTrackLeadDeg());
            telemetry.addData("Turret cmd", sotm.turretAngleDeg(shot));
        }

        telemetry.addData("Turret", getTurret().getAngle());
        telemetry.addData("Turret err", getTurret().getError());
        telemetry.addData("Flywheel", getShooter().Speed());
        telemetry.addData("Flywheel target", getShooter().getTargetVelocity());
        telemetry.addData("Ready", ready);
        telemetry.addData("Shots", getShooter().shotCount());
        telemetry.addData("Battery", getShooter().getVoltage());
        telemetry.addData("Aim shortfall", getTurret().getShortfall());
        telemetry.addData("Lateral Pointing To: ", sotm.lateral());
        telemetry.update();

        Drawing.drawDebug(follower);

        Drawing.drawPose(
                goal(), Drawing.POSE_LOOK
        );

        if (hasShot) Drawing.drawOffset(pose, new Pose(shot.aimX, shot.aimY));
    }

    private Pose diffPose;
    private long diffNs;
    private double fdVx, fdVy;

    private void speedCheck(Pose pose, Velocity vel) {
        long now = System.nanoTime();
        if (diffPose != null) {
            double dt = (now - diffNs) * 1e-9;
            if (dt > 1e-3) {
                fdVx += 0.2 * ((pose.x() - diffPose.x()) / dt - fdVx);
                fdVy += 0.2 * ((pose.y() - diffPose.y()) / dt - fdVy);
            }
        }
        diffPose = pose;
        diffNs = now;

        double fd = Math.hypot(fdVx, fdVy), pd = Math.hypot(vel.vx, vel.vy);
        telemetry.addData("Velocity pedro", "%.1f, %.1f (%.1f)", vel.vx, vel.vy, pd);
        telemetry.addData("Velocity pose-diff", "%.1f, %.1f (%.1f)", fdVx, fdVy, fd);
        telemetry.addData("Velocity ratio pedro/diff", fd > 8 ? pd / fd : Double.NaN);
    }

    private void logShot() {
        if (shot == null || !shot.valid) return;

        String line = String.format(Locale.US, "table.add(%.1f, %.0f, %.2f);", shot.distance, manualVelocity, ShotTable.ballisticTof(shot.distance));

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