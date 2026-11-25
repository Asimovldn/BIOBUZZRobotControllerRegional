package org.firstinspires.ftc.teamcode.systems.sotm;

import static org.firstinspires.ftc.teamcode.util.Util.compare;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;

public class ShotSolver {
    public static class Solution {
        public boolean valid;
        public double fieldAngle;
        public double launchHeading;
        public double distance;
        public double velocity, hood, tof;
        public double aimX, aimY;
        public double leadIn, latencyIn;
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
        public static boolean ballisticTof = true;
        public static double lateralGain = -0.06;
        public static double maxLateralShift = 20;
        public static double maxLeadIn = 40;
        public static double rightX = -1;
        public static double rightY = 0;
        public static double velocityGain = 1;
    }

    /* TODO: quando andando de lado em máxima velocidade, a mira está até correta,
    mas falta velocidade na bola, averiguar do porque dessa resenha */

    private static double tofFor(ShotTable.Entry e, double distance) {
        return SolverTune.ballisticTof ? ShotTable.ballisticTof(distance) : e.tof;
    }

    public ShotSolver(ShotTable table) {
        this.table = table;
    }

    public double getVelocityMultiplier(Pose pose, Pose goal, double vx, double vy) {
        Vector2D direction = (goal.toVector2D().minus(pose.toVector2D())).normalized();
        Vector2D velocity =  Vector2D.cartesian(vx,vy);

        double goalDirectionSpeed = direction.dot(velocity);

        return goalDirectionSpeed * SOTM.Tune.speedCorrectionMultiplier;
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
            double lx = svx * tof * SolverTune.velocityGain;
            double ly = svy * tof * SolverTune.velocityGain;
            double len = Math.hypot(lx, ly);
            if (len > SolverTune.maxLeadIn) {
                lx *= SolverTune.maxLeadIn / len;
                ly *= SolverTune.maxLeadIn / len;
            }

            ax = tx - lx;
            ay = ty - ly;
            d = Math.hypot(ax - sx, ay - sy);
            e = table.get(d);
            tof += damp * (tofFor(e, d) - tof);
        }

        Solution s = new Solution();
        s.valid = d > 1e-6;
        s.fieldAngle = Math.atan2(ay - sy, ax - sx);
        s.launchHeading = h;
        s.distance = d;
        s.velocity = e.velocity + Math.abs(e.velocity * getVelocityMultiplier(new Pose(x, y), new Pose(ax, ay), vx, vy));
        s.hood = e.hood;
        s.tof = tof;
        s.aimX = ax;
        s.aimY = ay;
        s.leadIn = Math.hypot(ax - tx, ay - ty);
        s.latencyIn = Math.hypot(vx, vy) * latency;
        return s;
    }
}