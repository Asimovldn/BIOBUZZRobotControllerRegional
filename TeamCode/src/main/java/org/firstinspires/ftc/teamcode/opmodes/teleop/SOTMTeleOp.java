package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Context;
import org.firstinspires.ftc.teamcode.RobotOpMode;
import org.firstinspires.ftc.teamcode.systems.hardware.Turret;
import org.firstinspires.ftc.teamcode.systems.sotm.SOTM;
import org.firstinspires.ftc.teamcode.systems.sotm.ShotSolver;

import java.io.FileWriter;
import java.io.IOException;

@TeleOp(name = "SOTM Red", group = "TeleOp")
public class SOTMTeleOp extends RobotOpMode {
    private static final double READY_TURRET_DEG = 1.5;

    protected final SOTM sotm = new SOTM();
    protected boolean manualOnly = false;

    private ShotSolver.Solution shot;
    private boolean sotmEnabled = true, spinEnabled = false;
    private double manualVelocity = 1300;

    /** Only used when the runtime has no pose yet (OpMode started without an Auto before it). */
    public Pose startPose() {
        return new Pose(59, 8, Math.toRadians(90));
    }

    @Override
    public void create() {
        getTurret().reset();
        if (runtime.pose == null) getFollower().setPose(startPose());

        fill();
    }

    public void fill() {
        sotm.table.add(85, 1300, 0, 1);
        sotm.table.add(33, 1100, 0, 0.65);
        sotm.table.add(73, 1250, 0, 0.4);
        sotm.table.add(54, 1150, 0, 0.8);
    }

    @Override
    public void setup() {
        Scheduler.reset();
        sotm.reset();
        getTurret().setMode(Turret.Mode.TRACK);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void periodic() {
        Pose pose = follower.pose();
        sotm.update(pose);

        Pose goal = goal();
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

        if (gamepad2.dpadUpWasPressed()) manualVelocity += 50;
        if (gamepad2.dpadDownWasPressed()) manualVelocity -= 50;
        if (gamepad2.leftTriggerWasPressed()) spinEnabled = !spinEnabled;
        if (gamepad2.leftTriggerWasReleased()) spinEnabled = !spinEnabled;
        if (gamepad2.dpadLeftWasPressed()) manualOnly = !manualOnly;

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
            gamepad2.rumbleBlips(2);

            try (FileWriter f = new FileWriter("table.txt", true)) {
                f.write("Distance: " + String.valueOf(shot.distance) + ", Velocity: " + String.valueOf(shot.velocity) + ";\n");
            } catch (IOException ignored) {}
        }

        if (gamepad2.bWasPressed()) {
            runtime.switchLocation();
            gamepad2.rumbleBlips(3);
        }

        drive(gamepad1);

        telemetry.addData("SOTM", sotmEnabled);
        telemetry.addData("Table", sotm.table.isEmpty() ? "EMPTY (fallback ToF, manual velocity)" : sotm.table.size() + " rows");
        telemetry.addData("Location", runtime.location);
        telemetry.addData("Pose", pose);
        telemetry.addData("Speed (in/s)", sotm.speed());
        telemetry.addData("Omega (rad/s)", sotm.omega());
        telemetry.addData("Servo ", getTurret().getServoPosition());

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
    }

    private Pose goal() {
        try {
            return new Context.HiveLocations(runtime).get();
        } catch (Exception e) {
            return null;
        }
    }
}
