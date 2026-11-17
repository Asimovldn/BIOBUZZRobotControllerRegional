package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;

/** Odometry reset poses. Named presets are red, mirror them for blue. Wall resets are alliance independent. */
public class ResetPoses {
    public static final double FIELD_SIZE = 144;

    @Config
    public static class Robot {
        /** Distance from the odometry tracking point to the rear edge of the robot, inches. */
        public static double rearIn = 7.5;
        /** Distance from the odometry tracking point to the front edge of the robot, inches. */
        public static double frontIn = 7.5;
    }

    public enum Wall { AUDIENCE, SCORING, LEFT, RIGHT }

    public static final Pose AUDIENCE_START = new Pose(59, 8, Math.toRadians(90));
    public static final Pose SCORING_CORNER = new Pose(10, 132, Math.toRadians(270));
    public static final Pose GARDEN = new Pose(12.7, 8.2, Math.toRadians(180));
    public static final Pose AUDIENCE_FLOWER = new Pose(11.7, 49.5, Math.toRadians(180));
    public static final Pose SCORING_FLOWER = new Pose(49.1, 131.7, Math.toRadians(90));
    public static final Pose PARK = new Pose(11.4, 114, Math.toRadians(0));

    private ResetPoses() {}

    /** Rotates a red pose 180 degrees around the field center, same as Auto.mirrorPose. */
    public static Pose mirror(Pose red) {
        return new Pose(FIELD_SIZE - red.x(), FIELD_SIZE - red.y(), Util.wrapRad(red.heading() + Math.PI));
    }

    public static Pose forAlliance(Pose red, boolean blue) {
        return blue ? mirror(red) : red;
    }

    /**
     * Robot backed flat against a wall and facing away from it: fixes heading and the axis
     * perpendicular to the wall, keeps the other coordinate from the current pose.
     */
    public static Pose wall(Pose current, Wall wall) {
        switch (wall) {
            case AUDIENCE:
                return new Pose(current.x(), Robot.rearIn, Math.toRadians(90));
            case SCORING:
                return new Pose(current.x(), FIELD_SIZE - Robot.rearIn, Math.toRadians(270));
            case LEFT:
                return new Pose(Robot.rearIn, current.y(), Math.toRadians(0));
            default:
                return new Pose(FIELD_SIZE - Robot.rearIn, current.y(), Math.toRadians(180));
        }
    }

    /** Same as {@link #wall} but with the front of the robot against the wall, so heading faces the wall. */
    public static Pose wallFront(Pose current, Wall wall) {
        switch (wall) {
            case AUDIENCE:
                return new Pose(current.x(), Robot.frontIn, Math.toRadians(270));
            case SCORING:
                return new Pose(current.x(), FIELD_SIZE - Robot.frontIn, Math.toRadians(90));
            case LEFT:
                return new Pose(Robot.frontIn, current.y(), Math.toRadians(180));
            default:
                return new Pose(FIELD_SIZE - Robot.frontIn, current.y(), Math.toRadians(0));
        }
    }
}
