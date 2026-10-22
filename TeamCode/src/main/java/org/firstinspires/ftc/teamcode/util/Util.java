package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Util {

    public static Pose fromFTCPose(Pose2D ftcPose) {
        return new Pose(
                ftcPose.getY(DistanceUnit.INCH) + 72,
                72 - ftcPose.getX(DistanceUnit.INCH),
                ftcPose.getHeading(AngleUnit.RADIANS) - Math.PI / 2
        );
    }

    public static double ticksPerDegree(double ticks) {
        return ticks / 360;
    }

    public static Pose fromFTCPose(Pose3D pose3d) {
        Position pos = pose3d.getPosition();
        YawPitchRollAngles orientation = pose3d.getOrientation();

        double x = pos.toUnit(DistanceUnit.INCH).x;
        double y = pos.toUnit(DistanceUnit.INCH).y;
        double heading = orientation.getYaw(AngleUnit.RADIANS);

        return new Pose(
                y + 72,
                72 - x,
                heading - Math.PI / 2
        );
    }

    public static Pose2D toPose2D(Pose3D pose3d) {
        Position pos = pose3d.getPosition();
        YawPitchRollAngles orientation = pose3d.getOrientation();

        return new Pose2D(
                pos.unit,
                pos.x,
                pos.y,
                AngleUnit.RADIANS,
                orientation.getYaw(AngleUnit.RADIANS)
        );
    }

    public static double normalizeAround(double angle, double center) {
        while (angle > center + 180) {
            angle -= 360;
        }

        while (angle < center - 180) {
            angle += 360;
        }

        return angle;
    }

    /**
     * Given the max/min KP, current/max ERR, estimate the best possible err.
     */
    public static double requiredKpValue(double maxKp, double minKp, double currErr, double maxErr) {
        double minErr = -maxErr;
        double tFactor = (currErr - minErr) / (maxErr - minErr);

        return lerp(minKp, maxKp, tFactor);
    }

    public static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    public static double wrapDeg(double deg) {
        double r = (deg + 180) % 360;
        if (r < 0) r += 360;
        return r - 180;
    }

    public static double wrapRad(double rad) {
        double r = (rad + Math.PI) % (2 * Math.PI);
        if (r < 0) r += 2 * Math.PI;
        return r - Math.PI;
    }

    public static boolean compare(Object a, Object b) { return a.equals(b); }

    public static double normalize(double t, double a, double b) {
        return (t - a) / (b - a);
    }

    public static double max(double[] vals) {
        double b = 0;
        for (double v : vals) {
            if (Math.abs(v) > Math.abs(b)) {
                b = v;
            }
        }

        return b;
    }
}
