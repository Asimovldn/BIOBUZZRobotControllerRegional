package org.firstinspires.ftc.teamcode.systems.sotm;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.util.Util;

import com.pedropathing.math.Velocity;

public class SOTM {
    @Config
    public static class Tune {
        public static double biasGain = 0.25;
        public static double maxBias = 30;
        public static double stationarySpeed = 4;
        public static double stationaryOmega = 0.15;
    }

    public final ShotTable table = new ShotTable();
    public final ShotSolver solver = new ShotSolver(table);

    /** turret + angle is clockwise, same as a positive Limelight tx. */
    public boolean clockwisePositive = true;

    /** velocity filter time constant, seconds. */
    public double tau = 0.06;

    private static final double MAX_SPEED = 200, MAX_OMEGA = 30;

    private double lastX, lastY, lastH;
    private double vx, vy, omega;
    private long lastNs;
    private boolean primed;
    private double bias;

    public void reset() {
        primed = false;
        vx = vy = omega = 0;
    }

    public void resetBias() {
        bias = 0;
    }

    public double bias() {
        return bias;
    }

    /** visionAngle: turret angle (deg) the Limelight says points at the goal. Only learns while stationary. */
    public boolean learnBias(double visionAngle, Pose pose, Pose goal) {
        if (Tune.biasGain <= 0) return false;
        if (speed() > Tune.stationarySpeed || Math.abs(omega) > Tune.stationaryOmega) return false;

        double ideal = Util.wrapDeg(visionAngle - directTurretAngleDeg(pose, goal));
        if (Math.abs(ideal) > Tune.maxBias) return false;

        bias += Tune.biasGain * (ideal - bias);
        return true;
    }

    public double directTurretAngleDeg(Pose pose, Pose goal) {
        ShotSolver.Solution s = solver.solve(pose.x(), pose.y(), pose.heading(), 0, 0, 0, goal.x(), goal.y());
        return rawTurretAngleDeg(s);
    }

    public void update(Pose pose) {
        update(pose.x(), pose.y(), pose.heading(), System.nanoTime());
    }

    public void update(Pose pose, Velocity velocity) {
        lastX = pose.x();
        lastY = pose.y();
        lastH = pose.heading();
        lastNs = System.nanoTime();
        primed = true;

        vx = velocity.vx;
        vy = velocity.vy;
        omega = velocity.omega;
    }

    public void update(double x, double y, double heading, long nowNs) {
        if (!primed) {
            lastX = x; lastY = y; lastH = heading; lastNs = nowNs;
            primed = true;
            return;
        }

        double dt = (nowNs - lastNs) * 1e-9;
        if (dt < 1e-3) return;

        double rvx = (x - lastX) / dt;
        double rvy = (y - lastY) / dt;
        double rw = Util.wrapRad(heading - lastH) / dt;

        if (Math.hypot(rvx, rvy) > MAX_SPEED || Math.abs(rw) > MAX_OMEGA) {
            vx = vy = omega = 0;
        } else {
            double a = 1 - Math.exp(-dt / tau);
            vx += a * (rvx - vx);
            vy += a * (rvy - vy);
            omega += a * (rw - omega);
        }

        lastX = x; lastY = y; lastH = heading; lastNs = nowNs;
    }

    public ShotSolver.Solution solve(Pose pose, Pose goal) {
        return solver.solve(pose.x(), pose.y(), pose.heading(), vx, vy, omega, goal.x(), goal.y());
    }

    public double turretAngleDeg(ShotSolver.Solution s) {
        return Util.wrapDeg(rawTurretAngleDeg(s) + bias);
    }

    private double rawTurretAngleDeg(ShotSolver.Solution s) {
        double rel = clockwisePositive ? s.launchHeading - s.fieldAngle : s.fieldAngle - s.launchHeading;
        return Util.wrapDeg(Math.toDegrees(rel));
    }

    public double vx() { return vx; }
    public double vy() { return vy; }
    public double omega() { return omega; }
    public double speed() { return Math.hypot(vx, vy); }
}
