package org.firstinspires.ftc.teamcode.storage;

import static com.pedropathing.utils.Utils.lerp;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.util.LazyMath;
import org.jetbrains.annotations.Nullable;

import java.lang.reflect.Array;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

import static org.firstinspires.ftc.teamcode.util.LazyMath.compare;
import static org.firstinspires.ftc.teamcode.util.LazyMath.normalize;

public class Storage {
    public static Pose autonomousEnd = new Pose(0, 0, 0); // TODO: change this later
    public static double targetBoost = 0;

    public static HashMap<Integer, Double> tagMap = new HashMap<>(16);

    public static class ShooterConstants {

        public static class BackShooter {
            public double kP = 0.01, kI = 0.001, kD = 0.005, kS = 0.16, kV = 0.0000571;
        }

        public static class ForeShooter {
            public double kP = 0.01, kI = 0.001, kD = 0.005, kS = 0.16, kV = 0.0000571;
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

        public static class ShootingUnit {
            public final double ta, multiplier, hoodAngle;
            public ShootingUnit(double targetArea, double mult, double hood) {
                ta = targetArea; multiplier = mult; hoodAngle = hood;
            }

            /** for templates */
            public ShootingUnit() {
                ta = 0; multiplier = 0; hoodAngle = 0;
            }

            public List<Double> asList() {
                return List.of(ta, multiplier, hoodAngle);
            }
        }

        public static class OdometryUnit {
            public final Pose pose;
            public final double multiplier, hoodAngle;

            public OdometryUnit(Pose pos, double m, double hood) {
                pose = pos; multiplier = m; hoodAngle = hood;
            }

            public OdometryUnit() {
                pose = Pose.zero(); multiplier = 0; hoodAngle = 0;
            }
        }

        public ShootingUnit[] table = {
                new ShootingUnit()
        };

        public OdometryUnit[] withOdometry = {
                new OdometryUnit()
        };


        public ShootingUnit getUnit(double ta) {
            @Nullable
            ShootingUnit res = null;
            for (ShootingUnit un : table) {
                if (un.ta == ta) {
                    res = un;
                }
            }

            if (res == null) {
                ShootingUnit[] closest = getClosestUnits(ta);

                double alpha = normalize(ta, closest[0].ta, closest[1].ta);
                double multiplier = lerp(closest[0].multiplier, closest[1].multiplier, alpha);
                double hoodAngle = lerp(closest[0].hoodAngle, closest[1].hoodAngle, alpha);

                res = new ShootingUnit(ta, multiplier, hoodAngle);
            }

            return res;
        }

        public ShootingUnit[] getClosestUnits(double ta) {
            ShootingUnit last = new ShootingUnit();
            ShootingUnit[] res = new ShootingUnit[2];
            for (ShootingUnit un : table) {
                if (ta < un.ta && ta > last.ta) {
                    res[0] = last;
                    res[1] = un;
                }

                last = un;
            }

            if (res[0] == null || res[1] == null) {
                throw new ArithmeticException("Cannot find closest units, please, insert more units inside the Look Up Table");
            }

            return res;
        }

        public OdometryUnit getUnit(Pose pos) {
            @Nullable
            OdometryUnit res = null;
            for (OdometryUnit un : withOdometry) {
                if (un.pose == pos) {
                    res = un;
                }
            }

            if (res == null) {
                OdometryUnit[] closest = getClosestUnits(pos);

                Pose a = closest[0].pose, b = closest[1].pose;

                double dx = b.x() - a.x();
                double dy = b.y() - a.y();

                double d = dx * dx + dy * dy;
                if (d == 0) return closest[0]; // both points are equal, somehow;

                double alpha = ((pos.x() - a.x()) * dx + (pos.y() - a.y()) * dy) / d;
                double multiplier = lerp(closest[0].multiplier, closest[1].multiplier, alpha);
                double hoodAngle = lerp(closest[0].hoodAngle, closest[1].hoodAngle, alpha);

                res = new OdometryUnit(pos, multiplier, hoodAngle);
            }

            return res;
        }

        public OdometryUnit[] getClosestUnits(Pose pos) {
            OdometryUnit last = new OdometryUnit();
            OdometryUnit[] res = new OdometryUnit[2];

            double smallDist = 9999;
            OdometryUnit smallUnit = new OdometryUnit(), altSmallUnit = new OdometryUnit();
            double altSmallDist = 9999;

            for (OdometryUnit un : withOdometry) {
                if (pos.distance(un.pose) < smallDist) {
                    altSmallDist = smallDist;
                    altSmallUnit = smallUnit;

                    smallDist = pos.distance(un.pose);
                    smallUnit = un;
                }
            }

            res[0] = smallUnit; res[1] = altSmallUnit;

            return res;
        }

        /**
         * Gets the unit from both Pose and the Target Area, getting their average
         * and utilizing that as a result. Good for precision giving both tunings.
         * @param pos Current Pose.
         * @param ta Limelight Target Area.
         * @return OdometryUnit with averaged multipler and hoodAngle values.
         */
        public OdometryUnit fromBoth(Pose pos, double ta) {
            ShootingUnit fromTA = getUnit(ta);
            OdometryUnit fromPose = getUnit(pos);

            double mult = (fromTA.multiplier + fromPose.multiplier) / 2;
            double angle = (fromTA.hoodAngle + fromPose.hoodAngle) / 2;

            return new OdometryUnit(pos, mult, angle);
        }
    };

    public static class TurretConstants {
        // Custom PIDF (not NextFTC control)
        public double posKp = 0.008, posKi = 0.0005, posKd = 0.004;
        public double posKf = 1.0; // multiplies -botAngularVelocity for disturbance cancellation

        // Inner velocity loop, handed directly to NextMotor built-in velocity controller
        public double velKp = 0.01, velKi = 0.002, velKd = 0.0;
        public double velKs = 0.08, velKv = 0.00242, velKa = 0.0;

        public int MAX_ANGLE = 300;
        public int WRAP_CENTER = 0, WRAP_TOLERANCE = 10;
        public int MOTOR_GEAR = 20, TURRET_GEAR = 40;
        public double GEAR_RATIO;

        public String motorName = "turret";

        // em tese use angleTolerance = 2, mas pode chutar qualquer valor sei lá.
        public double angleTolerance = 7;


        public TurretConstants() {
            GEAR_RATIO = (double) TURRET_GEAR / MOTOR_GEAR;
        }

        public void fillTagMap() {
            tagMap.put(34, -16.5);
            tagMap.put(35, -7.0);
            tagMap.put(36, 7.0);
            tagMap.put(37, 16.5);
        }

        public TurretConstants(double MAX_VELOCITY) {
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

    public static class IntakeConstants {
        public double kS = 0.16, kV = 0.0000571;

        public IntakeConstants() {};


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
