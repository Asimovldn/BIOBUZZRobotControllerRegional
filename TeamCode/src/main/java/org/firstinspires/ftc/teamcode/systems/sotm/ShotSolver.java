package org.firstinspires.ftc.teamcode.systems.sotm;

import static org.firstinspires.ftc.teamcode.util.Util.compare;

import com.acmerobotics.dashboard.config.Config;

public class ShotSolver {
    public static class Solution {
        public boolean valid;
        public double fieldAngle;
        public double launchHeading;
        public double distance;
        public double velocity, hood, tof;
        public double aimX, aimY;
    }

    public final ShotTable table;

    /** seconds between the pose sample and the ball leaving the shooter. */
    public double latency = 0.06, lateral;

    /** inches */
    public double offsetX = -5, offsetY = 0;

    public int iterations = 16;

    public double damp = 0.5;


    @Config
    public static class SolverTune {
        public static double lateralGain = -0.06;
        public static double maxLateralShift = 20;
        public static double rightX = -1;
        public static double rightY = 0;
        public static boolean ballisticTof = true;
    }

    private static double tofFor(ShotTable.Entry e, double distance) {
        return SolverTune.ballisticTof ? ShotTable.ballisticTof(distance) : e.tof;
    }

    public ShotSolver(ShotTable table) {
        this.table = table;
    }

    public Solution solve(double x, double y, double heading, double vx, double vy, double omega,
                          double goalX, double goalY, String location) {
        double h = heading + omega * latency;
        double cx = x + vx * latency;
        double cy = y + vy * latency;

        double cos = Math.cos(h), sin = Math.sin(h);
        double rx = offsetX * cos - offsetY * sin;
        double ry = offsetX * sin + offsetY * cos;

        double sx = cx + rx, sy = cy + ry;
        double svx = vx - omega * ry;
        double svy = vy + omega * rx;

        double isAudience = compare(location, "Audience") ? 1 : -1;
        lateral = ((sx - goalX) * SolverTune.rightX + (sy - goalY) * SolverTune.rightY) * isAudience;
        double shift = Math.max(-SolverTune.maxLateralShift, Math.min(SolverTune.maxLateralShift, -SolverTune.lateralGain * lateral));
        double tx = goalX + SolverTune.rightX * shift;
        double ty = goalY + SolverTune.rightY * shift;

        double ax = tx, ay = ty;
        double d = Math.hypot(ax - sx, ay - sy);
        ShotTable.Entry e = table.get(d);
        double tof = tofFor(e, d);

        for (int i = 0; i < iterations; i++) {
            ax = tx - svx * tof;
            ay = ty - svy * tof;
            d = Math.hypot(ax - sx, ay - sy);
            e = table.get(d);
            tof += damp * (tofFor(e, d) - tof);
        }

        Solution s = new Solution();
        s.valid = d > 1e-6;
        s.fieldAngle = Math.atan2(ay - sy, ax - sx);
        s.launchHeading = h;
        s.distance = d;
        s.velocity = e.velocity;
        s.hood = e.hood;
        s.tof = tof;
        s.aimX = ax;
        s.aimY = ay;
        return s;
    }
}