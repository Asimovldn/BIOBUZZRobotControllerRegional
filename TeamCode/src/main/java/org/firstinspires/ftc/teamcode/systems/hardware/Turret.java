package org.firstinspires.ftc.teamcode.systems.hardware;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Context;
import org.firstinspires.ftc.teamcode.systems.hardware.core.Mechanism;

import java.util.List;

/* TODO: Fix turret wobbling
        Solutions Proposed:
        Add tolerance (doesn't fix wobbling and makes the turret not correct sometimes).
        Reduce aim gain (0.15 made it not spin, 0.3 is too much wobble).
 */

/**
 * Turret driven by ONE servo in POSITIONAL mode.
 * SLEW = limited speed.<br>
 *<br>
 * Because there is no feedback: power on with the turret at its CENTER (servo jumps to 0.5 at start).
 *<br><br>
 * Modes:<br>
 *  OFF           ==> PWM disabled (servo limp).<br>
 *  CALIBRATE     ==> servo parked at 0.5. Use it to mount the horn / gear so the turret points forward.<br>
 *  MANUAL        ==> setManualVelocity(deg/s) moves the turret. Check the direction first.<br>
 *  SEMI_AIM      ==> Limelight aim, any tag, NO wrapping (just limits). For testing.<br>
 *  AIMING        ==> Limelight aim on goal tags, WITH wrapping into the safe range.<br>
 *  RECENTERING   ==> go to the center.<br>
 *  TUNE_POSITION ==> go to setManualAngle(deg).<br>
 *  <br>
 * Combine this with different OpModes for Turret Testing.<br>
 * change modes with:<br>
 * <pre>{@code
 *  turret.setMode(Mode m);
 * }
 */
public class Turret extends Mechanism {

    private static final String SERVO_NAME = "turret";

    private static final int SERVO_RANGE_DEG = 300;

    /** Turret degrees per servo degree. 1.0 = direct drive. 1/3 = servo gear 20T driving turret 60T. */
    private static final double TURRET_PER_SERVO = 1.0;

    private static final double PWM_MIN_US = 500, PWM_MAX_US = 2500;

    /** Stay away from the physical ends of the servo. */
    private static final double SERVO_MIN_POS = 0.005, SERVO_MAX_POS = 0.995;

    /** i don't know why I made this it will always be true we are using gears */
    private static final boolean SERVO_REVERSED = true;

    /** unprobable since we won't be inverting our limelight so whatever */
    private static final double AIM_SIGN = 1.0;

    /** lower this like it is your KP */
    private double AIM_GAIN = 0.12;

    /** Ignore tx smaller than this (deg) */
    private static final double AIM_DEADBAND_DEG = 7;

    /** Max turret speed (deg/s)  */
    private static final double MAX_SLEW_DPS = 190.0;

    /** Estimated time to let the turret settle */
    private static final double SERVO_SETTLE = 0.13; // in seconds

    /** variables made to support my bad code */
    private double lastMoveNs = 0, lastServoPos = 0.5;

    public String currentLocation = "Audience";

    /** no use besides debugging */
    private double deltatime = 0;

    public void editGain(double x) {
        AIM_GAIN = x;
    }

    public void switchLocation() {
        currentLocation = (String) Context.isEqualTo(currentLocation, "Audience", "Scoring");
    }

    public double isLimelighteando;

    public double getSpeed() {
        return 0.0; // placeholder
    }
    // =========================================

    public enum Mode { OFF, CALIBRATE, MANUAL, SEMI_AIM, AIMING, RECENTERING, TUNE_POSITION }

    private static final double TURRET_RANGE_DEG = SERVO_RANGE_DEG * TURRET_PER_SERVO;

    private final Context.TurretConstants constants = new Context.TurretConstants();
    private final ServoImplEx servo;
    private final Limelight3A limelight;

    private Context.Alliance alliance;
    private Mode currentMode = Mode.OFF;

    /** Modelled turret angle (deg) = last commanded angle. Center of the servo = WRAP_CENTER. */
    public double currentAngle, lastCurrentAngle;
    public boolean calledPeriodic = false;

    public int[] readingTag = new int[4];

    private double targetAngle;
    private double err = 0;
    private double lastWrappedTarget;
    private double lastResultTimestamp = -1;
    private double manualAngle = 0, manualVelocity = 0;
    private long lastNanos = 0;

    private double seenTa;

    private boolean hasRun = false;

    public Turret(Context runtime) {
        passInRuntime(runtime);
        HardwareMap hw = runtime.hardwareMap;

        servo = hw.get(ServoImplEx.class, SERVO_NAME);
        servo.setPwmRange(new PwmControl.PwmRange(PWM_MIN_US, PWM_MAX_US));

        servo.setPwmEnable();

        servo.setPosition(0.5); // i hope this is centered please let this be centered

        limelight = hw.get(Limelight3A.class, "Ethernet Device");


        currentAngle = constants.WRAP_CENTER; // we assume the turret starts centered
        lastCurrentAngle = currentAngle;
        targetAngle = currentAngle;
        lastWrappedTarget = currentAngle;

        reset();
    }

    // ------------------------------------------------------

    public void reset() {
        servo.setPwmDisable();

        limelight.stop();
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100);
        limelight.start();

        servo.setPosition(0.5);

        limelight.reloadPipeline();

        err = 0;
        currentAngle = 0;
        targetAngle = 0;

        alliance = runtime.GlobalAlliance;
        currentLocation = runtime.location;

        servo.setPwmEnable();
    }

    /**
     * Sets the Turret Mode<br>
     *    OFF           ==> PWM disabled (servo limp).<br>
     *    CALIBRATE     ==> servo parked at 0.5. Use it to mount the horn / gear so the turret points forward.<br>
     *    MANUAL        ==> setManualVelocity(deg/s) moves the turret. Check the direction first.<br>
     *    SEMI_AIM      ==> Limelight aim, any tag, NO wrapping (just limits). For testing.<br>
     *    AIMING        ==> Limelight aim on goal tags, WITH wrapping into the safe range.<br>
     *    RECENTERING   ==> go to the center.<br>
     *    TUNE_POSITION ==> go to setManualAngle(deg).<br>
     *    <br>
     *
     * @param m new mode
     */
    public void setMode(Mode m) {
        if (m == currentMode) return;
        currentMode = m;
        targetAngle = currentAngle; // never start a mode chasing an old target
        lastWrappedTarget = currentAngle;
        manualVelocity = 0;
        hasRun = false;
    }

    public boolean chamouSim = false;

    public Mode getMode() { return currentMode; }
    public void setManualAngle(double deg) { manualAngle = deg; }
    public void setManualVelocity(double dps) { manualVelocity = dps; }
    public double getAngle() { return currentAngle; }
    public double getError() { return err; }
    public double getTargetAngle() { return targetAngle; }
    public double getServoPosition() { return angleToServo(currentAngle); }
    public Context.Alliance getAlliance() { return alliance; }
    public void setAlliance(Context.Alliance alliance) { this.alliance = alliance; }

    public void forceStop() {
        limelight.stop();
        setMode(Mode.OFF);
    }

    // ------------------------------------------------------------------ main loop

    @Override
    public void periodic() {
        calledPeriodic = true;

        long now = System.nanoTime();
        double dt = lastNanos == 0 ? 0 : Range.clip((now - lastNanos) * 1e-9, 0, 0.1);
        lastNanos = now;

        deltatime = dt * 1000 %.3f; // ms

        double goal = currentAngle;
        currentLocation = runtime.location;

        switch (currentMode) {
            case OFF:
                if (servo.isPwmEnabled()) servo.setPwmDisable();
                return;

            case CALIBRATE:
                if (!servo.isPwmEnabled()) servo.setPwmEnable();
                currentAngle = constants.WRAP_CENTER;
                servo.setPosition(0.5);
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

            case RECENTERING:
                goal = constants.WRAP_CENTER;
                break;

            case TUNE_POSITION:
                goal = manualAngle;
                break;
        }

        if (!servo.isPwmEnabled()) servo.setPwmEnable();

        goal = clampToLimits(goal);
        err = (goal - currentAngle);

        isLimelighteando = goal;

        // move the model toward the goal at a limited speed, then command it
        double maxStep = MAX_SLEW_DPS * dt;
        currentAngle = clampToLimits(currentAngle + Range.clip(err, -maxStep, maxStep));
        double pos = angleToServo(currentAngle);

        if (Math.abs(pos - lastServoPos) > 0.01) {
            servo.setPosition(pos);
            lastCurrentAngle = err;
            hasRun = true;
        }

        lastServoPos = servo.getPosition();
    }

    public double getTa() {
        return seenTa;
    }

    // ------------------------------------------------------------------ aiming

    /** Returns the goal angle. wrap = false is SEMI_AIM (clamped to the limits, no wrapping). */
    private double aim(boolean wrap) {
        LLResult res = limelight.getLatestResult();

        if (Math.abs(targetAngle - currentAngle) > 0.3) lastMoveNs = System.nanoTime();

        // Get a NEW target only from a new, fresh, valid result; otherwise hold the last absolute target.
        if (res.isValid() && res.getTimestamp() != lastResultTimestamp) {
            lastResultTimestamp = res.getTimestamp();

            double ageMs = res.getStaleness() + res.getCaptureLatency() + res.getTargetingLatency();
            long captureNs = System.nanoTime() - (long) (ageMs * 1e6);

            if (captureNs > lastMoveNs + (long) (SERVO_SETTLE * 1e9)) {
                Double tx = readTx(res, wrap);
                if (tx != null && Math.abs(tx) > AIM_DEADBAND_DEG) {
                    targetAngle = currentAngle + AIM_SIGN * AIM_GAIN * tx;
                }
            }
        }

        return wrap ? wrapToSafeRange(targetAngle) : clampToLimits(targetAngle);
    }

    // Aim with Odometry
    private double aim() {
        Pose pose = runtime.follower.pose();
        Context.HiveLocations hives = new Context.HiveLocations(runtime);

        try {
            Pose toAim = hives.get();
            toAim.distance(pose);
        } catch (Exception e) {
            return 0.0;
        }

        return wrapToSafeRange(targetAngle);
    }

    private void setTagRead(int id, int n) {
        readingTag[n] = id;
    }

    private void resetTagRead() {
        readingTag = new int[4];
    }

    /** SEMI_AIM: any tag (LLResult tx). AIMING: only the goal tags, each with its own tx, averaged. */
    private Double readTx(LLResult res, boolean onlyGoalTags) {
        if (!onlyGoalTags || alliance == null) return res.getTx();

        int[] goal = Context.TagFamilies.get(alliance, currentLocation);
        List<LLResultTypes.FiducialResult> fids = res.getFiducialResults();
        if (fids == null) return null;

        List<Integer> goalTags = List.of(goal[0], goal[1], goal[2], goal[3]);

        double sum = 0;
        int n = 0;

        resetTagRead();

        for (LLResultTypes.FiducialResult f : fids) {
            int id = f.getFiducialId();
            if (goalTags.contains(id)) {
                try {
                    setTagRead(id, n);
                    sum += f.getTargetXDegrees() + (Context.tagMap.get(id) / 2);
                    n++;
                    seenTa = f.getTargetArea();
                    break;
                } catch (NullPointerException err) {
                    setTagRead(id, n);
                    sum += f.getTargetXDegrees();
                    n++;
                    deltatime = -1;
                }
            };
        }

        return n > 0 ? sum / n : null;
    }

    public double getDeltatime() {
        return deltatime;
    }

    // ------------------------------------------------------------------ geometry helpers

    /** Turret angle (deg) -> servo position (0..1). */
    private double angleToServo(double angle) {
        double pos = 0.5 + (angle - constants.WRAP_CENTER) / TURRET_RANGE_DEG;
        return posForServo(pos);
    }


    private double posForServo(double pos) {
        pos = Range.clip(pos, SERVO_MIN_POS, SERVO_MAX_POS);
        return SERVO_REVERSED ? 1.0 - pos : pos;
    }

    /** Lowest reachable turret angle: servo limit, tightened by MAX_ANGLE from Storage. */
    private double minAngle() {
        double servoMin = constants.WRAP_CENTER + (SERVO_MIN_POS - 0.5) * TURRET_RANGE_DEG;
        return Math.max(servoMin, constants.WRAP_CENTER - constants.MAX_ANGLE / 2.0);
    }

    private double maxAngle() {
        double servoMax = constants.WRAP_CENTER + (SERVO_MAX_POS - 0.5) * TURRET_RANGE_DEG;
        return Math.min(servoMax, constants.WRAP_CENTER + constants.MAX_ANGLE / 2.0);
    }

    private double clampToLimits(double angle) {
        return Range.clip(angle, minAngle(), maxAngle());
    }

    /**
     * Wraps into [minAngle, maxAngle] (+-360). Only for the turret. Hysteresis keeps it from flipping
     * 360 deg every loop when the target sits at the limit.
     *
     * Must consider that: Servos don't have 360-ish range, wrap only works to try to keep aiming besides limits.
     */
    private double wrapToSafeRange(double angle) {
        double lo = minAngle(), hi = maxAngle();

        double wrapped = angle;
        while (wrapped > hi) wrapped -= 360;
        while (wrapped < lo) wrapped += 360;
        if (wrapped > hi) wrapped = clampToLimits(angle); // range < 360 and angle falls in the dead zone, out of servo limits

        double alt = wrapped + (wrapped < lastWrappedTarget ? 360 : -360);
        boolean inRange = alt >= lo && alt <= hi;
        boolean closerToLast = Math.abs(alt - lastWrappedTarget) + constants.WRAP_TOLERANCE
                < Math.abs(wrapped - lastWrappedTarget);
        if (inRange && closerToLast) wrapped = alt;

        lastWrappedTarget = wrapped;
        return wrapped;
    }
}