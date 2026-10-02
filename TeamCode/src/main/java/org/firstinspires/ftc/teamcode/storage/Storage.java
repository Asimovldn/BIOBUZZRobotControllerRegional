package org.firstinspires.ftc.teamcode.storage;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.util.LazyMath;

import java.lang.reflect.Array;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Map;
import java.util.Objects;

import static org.firstinspires.ftc.teamcode.util.LazyMath.compare;

public class Storage {
    public static Pose autonomousEnd = new Pose(0, 0, 0); // TODO: change this later
    public static double targetBoost = 0;

    public static class ShooterConstants {
        public double kP = 0.01, kI = 0.001, kD = 0.005, kS = 0.16, kV = 0.0000571;

        public void editSetting(String setting, Object v) {
            try {
                Field f = this.getClass().getField(setting);
                f.setAccessible(true);
                f.set(this, v);
            } catch (NoSuchFieldException | IllegalAccessException e) {
                throw new RuntimeException(e);
            };
        }
    };

    public static class TurretConstants {
        // Custom PIDF (not NextMotor control)
        public double posKp = 0.02, posKi = 0.0005, posKd = 0.004;
        public double posKf = 1.0; // multiplies -botAngularVelocity for disturbance cancellation

        // Inner velocity loop, handed directly to NextMotor built-in velocity controller
        public double velKp = 0.01, velKi = 0.002, velKd = 0.0;
        public double velKs = 0.05, velKv = 0.004, velKa = 0.0;

        public int MAX_ANGLE = 350;
        public int WRAP_CENTER = 0, WRAP_HYSTERESIS = 20;
        public int MOTOR_GEAR = 20, TURRET_GEAR = 40;
        public double GEAR_RATIO;

        public String motorName = "turretMotor";

        // em tese use angleTolerance = 2, mas pode chutar qualquer valor sei lá.
        public double angleTolerance = 7;

        public double MAX_VEL = 15; // degrees/sec, turret-referenced

        public TurretConstants() {
            GEAR_RATIO = (double) TURRET_GEAR / MOTOR_GEAR;
        }

        public TurretConstants(double MAX_VELOCITY) {
            MAX_VEL = MAX_VELOCITY;
            GEAR_RATIO = (double) TURRET_GEAR / MOTOR_GEAR;
        }

        public void editSetting(String setting, Object v) {
            try {
                Field f = this.getClass().getField(setting);
                f.setAccessible(true);
                f.set(this, v);
            } catch (NoSuchFieldException | IllegalAccessException e) {
                throw new RuntimeException(e);
            };
        }

        public double compress(double angle) {
            angle = ((angle + 180) % 360 + 360) % 360 - 180;
            return angle / 180;
        }
    };

    public void editSetting(String setting, Object v) {
        try {
            Field f = this.getClass().getField(setting);
            f.setAccessible(true);
            f.set(this, v);
        } catch (NoSuchFieldException | IllegalAccessException e) {
            throw new RuntimeException(e);
        };
    }

    private static int[] fillArray(int[] tags, int a, int b, int c, int d) {
        return new int[]{a, b, c, d};
    }

    public static class TagFamilies {
        private Alliance alliance;

        public static int[] get(Alliance alliance, String location) {
            int[] tags = new int[4];

            tags = fillArray(tags, 0, 0, 0, 0);

            if (alliance == Alliance.BLUE) {
                if (compare(location, "Audience")) {
                    tags = fillArray(tags, 38, 39, 40, 41);
                } else if (compare(location, "Scoring")) {
                    tags = fillArray(tags, 42, 43, 44, 45);
                }
            } else if (alliance == Alliance.RED) {
                if (compare(location, "Audience")) {
                    tags = fillArray(tags, 34, 35, 36, 37);
                } else if (compare(location, "Scoring")) {
                    tags = fillArray(tags, 30, 31, 32, 33);
                }
            }

            return tags;
        }

        public TagFamilies() {

        }

        public TagFamilies(Alliance alliance) {
            this.alliance = alliance;
        }

        public int[] get(String location) {
            return get(alliance, location);
        }
    }

    public static Alliance GlobalAlliance = Alliance.BLUE;

    public enum Alliance {
        BLUE, RED
    };

    /**
     * If {@code ths} is equal to {@code equal} then turn into {@code then} else turn into {@code elseThen}
     * @param ths this
     * @param equal equal to this
     * @param then become this
     * @param elseThen else...
     * @return then or elseThen
     */
    public static Object isEqualTo(Object ths, Object equal, Object then, Object elseThen) {
        if (ths.equals(equal)) {
            return then;
        } else {
            return elseThen;
        }
    }

    public static Object isEqualTo(Object ths, Object equal, Object then) {
        return isEqualTo(ths, equal, then, equal);
    }
}
