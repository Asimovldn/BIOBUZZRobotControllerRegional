package org.firstinspires.ftc.teamcode.systems.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Context;
import org.firstinspires.ftc.teamcode.systems.hardware.core.Mechanism;
import org.firstinspires.ftc.teamcode.util.Util;

import java.util.List;

/**
 * Turret driven by ONE servo in POSITIONAL mode, slew limited. No feedback: power on with the turret centered.
 * Angle is relative to the robot, + is clockwise (same sign as a positive Limelight tx).
 * <pre>
 *  OFF           PWM disabled (servo limp)
 *  CALIBRATE     servo parked at center
 *  MANUAL        setManualVelocity(deg/s)
 *  SEMI_AIM      Limelight aim on any tag, no wrapping
 *  AIMING        Limelight aim on goal tags, wrapping into the safe range
 *  TRACK         follows setTrackAngle(deg), wrapping into the safe range (odometry / SOTM)
 *  RECENTERING   go to the center
 *  TUNE_POSITION go to setManualAngle(deg)
 * </pre>
 */
public class Turret extends Mechanism {

    @Config
    public static class Tune {
        public static double aimGain = 0.7;
        public static double aimDeadbandDeg = 1.5;
        public static double settleSec = 0.08;

        /** Servo shaft speed in deg/s. The turret slews at this times turretPerServo. */
        public static double servoSpeedDps = 280.0;

        /** Turret degrees per servo degree. 1.0 = direct drive, 0.5 = servo gear drives a gear twice its size. */
        public static double turretPerServo = 1.0;

        /** just in case the engineering team messes with the gear positions, adjust by degree offset */
        public static double centerTrimDeg = 0.0;

        public static double trackLeadSec = 0.04;
    }

    private static final String SERVO_NAME = "turret";
    private static final int SERVO_RANGE_DEG = 320;

    private static final double PWM_MIN_US = 500, PWM_MAX_US = 2500;
    private static final double SERVO_MIN_POS = 0, SERVO_MAX_POS = 1;
    private static final double SERVO_WRITE_EPS = 0.001;
    private static final boolean SERVO_REVERSED = false;
    private static final double AIM_SIGN = 1.0;

    private static final int HISTORY = 64;

    public enum Mode { OFF, CALIBRATE, MANUAL, SEMI_AIM, AIMING, TRACK, RECENTERING, TUNE_POSITION }

    public String currentLocation = "Audience";
    public int[] readingTag = new int[4];
    public boolean calledPeriodic = false;

    public double currentAngle;

    private final Context.TurretConstants constants = new Context.TurretConstants();
    private final ServoImplEx servo;
    private final Limelight3A limelight;

    private Context.Alliance alliance;
    private Mode currentMode = Mode.OFF;

    private double targetAngle, err, lastWrappedTarget;
    private double lastResultTimestamp = -1;
    private double manualAngle = 0, manualVelocity = 0, trackAngle = 0, trackVel = 0;
    private double visionAngle;
    private boolean visionFresh = false;
    private long trackStampNs = 0;
    private double lastServoPos = 0.5;
    private double seenTa, deltatime;
    private long lastNanos = 0, lastMoveNs = 0;

    private final long[] histNs = new long[HISTORY];
    private final double[] histAngle = new double[HISTORY];
    private int histHead = 0, histCount = 0;

    public Turret(Context runtime) {
        passInRuntime(runtime);
        HardwareMap hw = runtime.hardwareMap;

        servo = hw.get(ServoImplEx.class, SERVO_NAME);
        servo.setPwmRange(new PwmControl.PwmRange(PWM_MIN_US, PWM_MAX_US));
        limelight = hw.get(Limelight3A.class, "Ethernet Device");

        reset();
    }

    public void reset() {
        limelight.stop();
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.reloadPipeline();

        servo.setPosition(0.5);
        lastServoPos = 0.5;

        currentAngle = center();
        targetAngle = currentAngle;
        lastWrappedTarget = currentAngle;
        trackAngle = currentAngle;
        trackVel = 0;
        trackStampNs = 0;
        visionFresh = false;
        err = 0;
        histCount = 0;
        histHead = 0;
        lastMoveNs = System.nanoTime() + 1_000_000_000L;

        alliance = runtime.GlobalAlliance;
        currentLocation = runtime.location;

        servo.setPwmEnable();
    }

    public void setMode(Mode m) {
        if (m == currentMode) return;
        currentMode = m;
        targetAngle = currentAngle;
        lastWrappedTarget = currentAngle;
        manualVelocity = 0;
        trackVel = 0;
        visionFresh = false;
    }

    public Mode getMode() { return currentMode; }
    public void setManualAngle(double deg) { manualAngle = deg; }
    public void setManualVelocity(double dps) { manualVelocity = dps; }

    public void setTrackAngle(double deg) {
        long now = System.nanoTime();

        if (trackStampNs != 0) {
            double dt = (now - trackStampNs) * 1e-9;
            double step = Util.wrapDeg(deg - trackAngle);

            if (dt >= 0.2) {
                trackVel = 0;
            } else if (dt > 0.002 && Math.abs(step) < 10) {
                trackVel += 0.3 * (step / dt - trackVel);
            }
        }

        trackStampNs = now;
        trackAngle = deg;
    }

    /** limelight as a safety use for turret unalign */
    public Double consumeVisionAngle() {
        if (!visionFresh) return null;

        visionFresh = false;
        return visionAngle;
    }

    public void editGain(double x) { Tune.aimGain = x; }
    public double getAngle() { return currentAngle; }
    public double getError() { return err; }
    public double getTargetAngle() { return targetAngle; }
    public double getServoPosition() { return angleToServo(currentAngle); }
    public double getTa() { return seenTa; }
    public double getDeltatime() { return deltatime; }
    public Context.Alliance getAlliance() { return alliance; }
    public void setAlliance(Context.Alliance alliance) { this.alliance = alliance; }

    public boolean isOnTarget(double toleranceDeg) {
        return Math.abs(err) < toleranceDeg;
    }

    public void switchLocation() {
        currentLocation = (String) Context.isEqualTo(currentLocation, "Audience", "Scoring");
    }

    public void forceStop() {
        limelight.stop();
        setMode(Mode.OFF);
    }

    @Override
    public void periodic() {
        calledPeriodic = true;

        long now = System.nanoTime();
        double dt = lastNanos == 0 ? 0 : Range.clip((now - lastNanos) * 1e-9, 0, 0.1);
        lastNanos = now;
        deltatime = dt * 1000;

        currentLocation = runtime.location;
        double goal = currentAngle;

        switch (currentMode) {
            case OFF:
                if (servo.isPwmEnabled()) servo.setPwmDisable();
                return;

            case CALIBRATE:
                if (!servo.isPwmEnabled()) servo.setPwmEnable();
                currentAngle = center();
                servo.setPosition(0.5);
                lastServoPos = 0.5;
                return;

            case MANUAL:
                goal = currentAngle + manualVelocity * dt;
                break;

            case SEMI_AIM:
                goal = aim(false);
                break;

            case AIMING:
                goal = aim(true);
                break;

            case TRACK:
                if (trackVel != 0 && now - trackStampNs > 200_000_000L) trackVel = 0;
                visionUpdate(now);

                goal = wrapToSafeRange(trackAngle + Range.clip(trackVel, -slewDps(), slewDps()) * Tune.trackLeadSec);
                targetAngle = goal;
                break;

            case RECENTERING:
                goal = center();
                break;

            case TUNE_POSITION:
                goal = manualAngle;
                break;
        }

        if (!servo.isPwmEnabled()) servo.setPwmEnable();

        goal = clampToLimits(goal);
        err = goal - currentAngle;

        double before = currentAngle;
        double maxStep = slewDps() * dt;
        if (Math.abs(err) <= maxStep) {
            currentAngle = goal;
        } else {
            currentAngle = clampToLimits(currentAngle + Math.signum(err) * maxStep);
        }

        if (Math.abs(currentAngle - before) > 0.02) lastMoveNs = now;

        double pos = angleToServo(currentAngle);
        boolean arrived = currentAngle == goal;
        if (Math.abs(pos - lastServoPos) > SERVO_WRITE_EPS || (arrived && pos != lastServoPos)) {
            servo.setPosition(pos);
            lastServoPos = pos;
        }

        record(now, currentAngle);
    }

    /** Returns the goal angle. wrap = false is SEMI_AIM (clamped to the limits, no wrapping). */
    private double aim(boolean wrap) {
        LLResult res = limelight.getLatestResult();

        if (Math.abs(targetAngle - currentAngle) > 0.3) lastMoveNs = System.nanoTime();

        if (res != null && res.isValid() && res.getTimestamp() != lastResultTimestamp) {
            lastResultTimestamp = res.getTimestamp();

            double ageMs = res.getStaleness() + res.getCaptureLatency() + res.getTargetingLatency();
            long captureNs = System.nanoTime() - (long) (ageMs * 1e6);

            if (captureNs > lastMoveNs + (long) (Tune.settleSec * 1e9)) {
                Double tx = readTx(res, wrap);
                if (tx != null && Math.abs(tx) > Tune.aimDeadbandDeg) {
                    targetAngle = angleAt(captureNs) + AIM_SIGN * Tune.aimGain * tx;
                }
            }
        }

        return wrap ? wrapToSafeRange(targetAngle) : clampToLimits(targetAngle);
    }

    /** updates for turre track */
    private void visionUpdate(long now) {
        LLResult res = limelight.getLatestResult();
        if (res == null || !res.isValid() || res.getTimestamp() == lastResultTimestamp) return;
        lastResultTimestamp = res.getTimestamp();

        double ageMs = res.getStaleness() + res.getCaptureLatency() + res.getTargetingLatency();
        long captureNs = now - (long) (ageMs * 1e6);
        if (captureNs <= lastMoveNs + (long) (Tune.settleSec * 1e9)) return;

        Double tx = readTx(res, true);
        if (tx == null) return;

        visionAngle = angleAt(captureNs) + tx * AIM_SIGN;
        visionFresh = true;
    }

    /** SEMI_AIM: any tag. AIMING: only the goal tags, each with its own offset, averaged. */
    private Double readTx(LLResult res, boolean onlyGoalTags) {
        if (!onlyGoalTags || alliance == null) return res.getTx();

        List<LLResultTypes.FiducialResult> fids = res.getFiducialResults();
        if (fids == null) return null;

        int[] goal = Context.TagFamilies.get(alliance, currentLocation);
        readingTag = new int[4];

        double sum = 0, bestTa = 0;
        int n = 0;

        for (LLResultTypes.FiducialResult f : fids) {
            int id = f.getFiducialId();
            if (!isGoalTag(goal, id)) continue;

            Double offset = Context.tagMap.get(id);
            sum += f.getTargetXDegrees() + (offset == null ? 0 : offset / 2);
            bestTa = Math.max(bestTa, f.getTargetArea());

            if (n < readingTag.length) readingTag[n] = id;
            n++;
        }

        if (n == 0) return null;

        seenTa = bestTa;
        return sum / n;
    }

    private static boolean isGoalTag(int[] goal, int id) {
        for (int g : goal) {
            if (g == id) return true;
        }
        return false;
    }

    private void record(long ns, double angle) {
        histNs[histHead] = ns;
        histAngle[histHead] = angle;
        histHead = (histHead + 1) % HISTORY;
        if (histCount < HISTORY) histCount++;
    }

    private double angleAt(long ns) {
        if (histCount == 0) return currentAngle;

        int newer = (histHead - 1 + HISTORY) % HISTORY;
        if (ns >= histNs[newer]) return histAngle[newer];

        for (int i = 1; i < histCount; i++) {
            int older = (newer - 1 + HISTORY) % HISTORY;
            if (ns >= histNs[older]) {
                double span = histNs[newer] - histNs[older];
                double t = span <= 0 ? 0 : (ns - histNs[older]) / span;
                return Util.lerp(histAngle[older], histAngle[newer], t);
            }
            newer = older;
        }

        return histAngle[newer];
    }

    private double center() {
        return constants.WRAP_CENTER + Tune.centerTrimDeg;
    }

    public static double ratio() {
        return Math.abs(Tune.turretPerServo);
    }

    private static double rangeDeg() {
        return SERVO_RANGE_DEG * ratio();
    }

    private static double slewDps() {
        return Tune.servoSpeedDps * ratio();
    }

    /** Turret angle (deg) -> servo position [0, 1]. */
    private double angleToServo(double angle) {
        double pos = 0.5 + ((angle - center()) / rangeDeg());
        return posForServo(pos);
    }

    private double posForServo(double pos) {
        pos = Range.clip(pos, SERVO_MIN_POS, SERVO_MAX_POS);
        return SERVO_REVERSED ? 1.0 - pos : pos;
    }

    private double minAngle() {
        double servoMin = center() + (SERVO_MIN_POS - 0.5) * rangeDeg();
        return Math.max(servoMin, center() - constants.MAX_ANGLE / 2.0);
    }

    private double maxAngle() {
        double servoMax = center() + (SERVO_MAX_POS - 0.5) * rangeDeg();
        return Math.min(servoMax, center() + constants.MAX_ANGLE / 2.0);
    }

    private double clampToLimits(double angle) {
        return Range.clip(angle, minAngle(), maxAngle());
    }

    /**
     * Wraps into [minAngle, maxAngle] (+-360). Hysteresis keeps it from flipping 360 deg every loop
     * when the target sits at the limit. The servo range is under 360, so targets in the dead zone are clamped.
     */
    private double wrapToSafeRange(double angle) {
        if (Double.isNaN(angle) || Double.isInfinite(angle)) return lastWrappedTarget;

        double lo = minAngle(), hi = maxAngle();

        double wrapped = angle;
        while (wrapped > hi) wrapped -= 360;
        while (wrapped < lo) wrapped += 360;
        if (wrapped > hi) wrapped = clampToLimits(angle);

        double alt = wrapped + (wrapped < lastWrappedTarget ? 360 : -360);
        boolean inRange = alt >= lo && alt <= hi;
        boolean closerToLast = Math.abs(alt - lastWrappedTarget) + constants.WRAP_TOLERANCE
                < Math.abs(wrapped - lastWrappedTarget);
        if (inRange && closerToLast) wrapped = alt;

        lastWrappedTarget = wrapped;
        return wrapped;
    }
}
