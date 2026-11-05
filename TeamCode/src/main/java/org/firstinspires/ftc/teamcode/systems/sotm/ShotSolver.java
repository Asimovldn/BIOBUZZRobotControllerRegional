package org.firstinspires.ftc.teamcode.systems.sotm;

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
    public double latency = 0.06;

    /** inches */
    public double offsetX = -5, offsetY = 0;

    public int iterations = 16;

    public double damp = 0.5;

    public double rightX = 1, rightY = 0;

    public double lateralGain = 0.1;

    public double maxLateralShift = 6;

    public ShotSolver(ShotTable table) {
        this.table = table;
    }

    public Solution solve(double x, double y, double heading, double vx, double vy, double omega,
                          double goalX, double goalY) {
        double h = heading + omega * latency;
        double cx = x + vx * latency;
        double cy = y + vy * latency;

        double cos = Math.cos(h), sin = Math.sin(h);
        double rx = offsetX * cos - offsetY * sin;
        double ry = offsetX * sin + offsetY * cos;

        double sx = cx + rx, sy = cy + ry;
        double svx = vx - omega * ry;
        double svy = vy + omega * rx;

        double lateral = (sx - goalX) * rightX + (sy - goalY) * rightY;
        double shift = Math.max(-maxLateralShift, Math.min(maxLateralShift, -lateralGain * lateral));
        double tx = goalX + rightX * shift;
        double ty = goalY + rightY * shift;

        double ax = tx, ay = ty;
        double d = Math.hypot(ax - sx, ay - sy);
        ShotTable.Entry e = table.get(d);
        double tof = e.tof;

        for (int i = 0; i < iterations; i++) {
            ax = tx - svx * tof;
            ay = ty - svy * tof;
            d = Math.hypot(ax - sx, ay - sy);
            e = table.get(d);
            tof += damp * (e.tof - tof);
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