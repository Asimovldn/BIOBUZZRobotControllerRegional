package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotOpMode;
import org.firstinspires.ftc.teamcode.systems.hardware.Shooter;
import org.firstinspires.ftc.teamcode.systems.hardware.Turret;

import java.util.ArrayList;
import java.util.List;

/**
 * A: spin / stop. D-pad up/down: target +-10 (left bumper: +-50). Right bumper: feed. X: reset stats.
 * Left bumper + Y: power sweep that measures kS and kV for Shooter.PowerTuning.
 */
@TeleOp(name = "Shooter Tune", group = "Test")
public class ShooterTune extends RobotOpMode {
    private static final double[] SWEEP = {0.25, 0.35, 0.45, 0.55, 0.65, 0.75, 0.85, 0.95};

    private final ElapsedTime clock = new ElapsedTime();
    private final List<double[]> points = new ArrayList<>();

    private double target = 1200;
    private boolean spin;

    private double riseMs = -1, peak, minWhileFeeding = Double.MAX_VALUE, dip, recoveryMs = -1;
    private int lastShots;
    private double recoveryStartMs = -1;

    private int sweepIdx = -1;
    private double sweepSum;
    private int sweepN;
    private String fit = "not run";

    @Override
    public void setup() {
        Scheduler.reset();
        getTurret().setMode(Turret.Mode.TRACK);
    }

    @Override
    public void periodic() {
        Shooter sh = getShooter();

        if (sweepIdx < 0) manualControl(sh);
        else sweep(sh);

        if (gamepad1.left_bumper && gamepad1.yWasPressed() && sweepIdx < 0) startSweep();

        double speed = sh.speed();
        if (spin && sweepIdx < 0) {
            peak = Math.max(peak, speed);
            if (riseMs < 0 && sh.isReady()) riseMs = clock.milliseconds();

            if (sh.isFeeding()) minWhileFeeding = Math.min(minWhileFeeding, speed);
            if (sh.shotCount() > lastShots) {
                lastShots = sh.shotCount();
                dip = target - minWhileFeeding;
                minWhileFeeding = Double.MAX_VALUE;
                recoveryStartMs = clock.milliseconds();
            }
            if (recoveryStartMs >= 0 && sh.isReady()) {
                recoveryMs = clock.milliseconds() - recoveryStartMs;
                recoveryStartMs = -1;
            }
        }

        telemetry.addData("Target", target);
        telemetry.addData("Speed", speed);
        telemetry.addData("Error", target - speed);
        telemetry.addData("Ready", sh.isReady());
        telemetry.addData("Battery", sh.getVoltage());
        telemetry.addData("Rise to ready (ms)", riseMs);
        telemetry.addData("Overshoot", spin ? peak - target : 0);
        telemetry.addData("Shots", sh.shotCount());
        telemetry.addData("Last dip (ticks/s)", dip);
        telemetry.addData("Recovery (ms)", recoveryMs);
        telemetry.addData("Sweep", sweepIdx < 0 ? fit : "step " + (sweepIdx + 1) + "/" + SWEEP.length);
        telemetry.update();
    }

    private void manualControl(Shooter sh) {
        double step = gamepad1.left_bumper ? 50 : 10;
        if (gamepad1.dpadUpWasPressed()) target += step;
        if (gamepad1.dpadDownWasPressed()) target -= step;
        target = Math.max(0, target);

        if (gamepad1.aWasPressed()) {
            spin = !spin;
            if (spin) resetStats();
        }

        if (gamepad1.xWasPressed()) resetStats();

        sh.spinTo(spin ? target : 0);
        sh.setFeeding(gamepad1.right_bumper);
        getIntake().setThrottle(gamepad1.right_bumper ? 1 : 0);
    }

    private void resetStats() {
        clock.reset();
        riseMs = -1;
        peak = 0;
        minWhileFeeding = Double.MAX_VALUE;
        dip = 0;
        recoveryMs = -1;
        recoveryStartMs = -1;
        getShooter().resetShotCount();
        lastShots = 0;
    }

    private void startSweep() {
        spin = false;
        getShooter().stopFlywheel();
        getShooter().setFeeding(false);
        points.clear();
        sweepIdx = 0;
        sweepSum = 0;
        sweepN = 0;
        clock.reset();
    }

    private void sweep(Shooter sh) {
        double t = clock.seconds();
        sh.setOpenLoopPower(SWEEP[sweepIdx]);

        if (t > 1.4) {
            sweepSum += sh.speed();
            sweepN++;
        }

        if (t < 2.0) return;

        double avg = sweepN == 0 ? 0 : sweepSum / sweepN;
        points.add(new double[]{avg, SWEEP[sweepIdx] * sh.getVoltage() / Shooter.Tuning.nominalVoltage});

        sweepIdx++;
        sweepSum = 0;
        sweepN = 0;
        clock.reset();

        if (sweepIdx >= SWEEP.length) {
            sweepIdx = -1;
            sh.setOpenLoopPower(null);
            fit = fitResult();
        }
    }

    private String fitResult() {
        double n = 0, sx = 0, sy = 0, sxx = 0, sxy = 0;
        for (double[] p : points) {
            if (p[0] < 50) continue;
            n++;
            sx += p[0];
            sy += p[1];
            sxx += p[0] * p[0];
            sxy += p[0] * p[1];
        }

        double den = n * sxx - sx * sx;
        if (n < 3 || Math.abs(den) < 1e-9) return "failed, not enough moving points";

        double kV = (n * sxy - sx * sy) / den;
        double kS = (sy - kV * sx) / n;

        return String.format("kS=%.4f kV=%.6f max=%.0f ticks/s", kS, kV, (1 - kS) / kV);
    }

    @Override
    public void stop() {
        if (getShooter() != null) getShooter().brakeNow();
    }
}
