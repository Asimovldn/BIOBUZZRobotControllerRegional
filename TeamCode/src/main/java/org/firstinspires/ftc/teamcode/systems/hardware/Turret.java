package org.firstinspires.ftc.teamcode.systems.hardware;

import static org.firstinspires.ftc.teamcode.util.LazyMath.compare;
import static dev.nextftc.units.Units.Degrees;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.storage.Storage;
import org.firstinspires.ftc.teamcode.util.LazyMath;

import java.util.List;

import dev.nextftc.hardware.webcams.NextLimelight;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.units.measuretypes.Angle;

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
public class Turret implements Mechanism {

    private static final String SERVO_NAME = "turret";

    private static final Angle SERVO_RANGE_DEG = Degrees.of(300);

    /** Turret degrees per servo degree. 1.0 = direct drive. 1/3 = servo gear 20T driving turret 60T. */
    private static final double TURRET_PER_SERVO = 0.5;

    private static final double PWM_MIN_US = 500, PWM_MAX_US = 2500;

    /** Stay away from the physical ends of the servo. */
    private static final double SERVO_MIN_POS = 0.01, SERVO_MAX_POS = 0.99;

    /** Flip if +angle moves the turret the wrong way. Probable if using gears, since they invert motion. */
    private static final boolean SERVO_REVERSED = true;

    /** Flip to -1 if Limelight tx>0 makes the turret aim away from the tag. */
    private static final double AIM_SIGN = 1.0;

    /** Fraction of tx corrected per new result (lower if it oscillates because of camera latency). */
    private static final double AIM_GAIN = 0.8;

    /** Ignore tx smaller than this (deg) */
    private static final double AIM_DEADBAND_DEG = 0.7;

    /** Max turret speed (deg/s). Keep under ~70% of servo speed * TURRET_PER_SERVO.  */
    private static final double MAX_SLEW_DPS = 300.0;

    private static String currentLocation = "Audience";

    public void switchLocation() {
        currentLocation = (String) Storage.isEqualTo(currentLocation, "Audience", "Scoring");
    }

    public double getSpeed() {
        return 0.0; // placeholder
    }
    // =========================================

    public enum Mode { OFF, CALIBRATE, MANUAL, SEMI_AIM, AIMING, RECENTERING, TUNE_POSITION }

    private static final Angle TURRET_RANGE_DEG = Degrees.of(SERVO_RANGE_DEG.into(Degrees) * TURRET_PER_SERVO);

    private final Storage.TurretConstants constants = new Storage.TurretConstants();
    private final ServoImplEx servo;
    private final NextLimelight limelight;

    private Storage.Alliance alliance;
    private Mode currentMode = Mode.OFF;

    /** Modelled turret angle (deg) = last commanded angle. Center of the servo = WRAP_CENTER. */
    public double currentAngle;
    public boolean calledPeriodic = false;

    private double targetAngle;
    private double err = 0;
    private double lastWrappedTarget;
    private double lastResultTimestamp = -1;
    private double manualAngle = 0, manualVelocity = 0;
    private long lastNanos = 0;

    public Turret(HardwareMap hw) {
        servo = hw.get(ServoImplEx.class, SERVO_NAME);
        servo.setPwmRange(new PwmControl.PwmRange(PWM_MIN_US, PWM_MAX_US));

        servo.setPosition(posForServo(0.5));

        limelight = new NextLimelight("Ethernet Device");
        limelight.startReading(0, 100);

        currentAngle = constants.WRAP_CENTER; // we assume the turret starts centered
        targetAngle = currentAngle;
        lastWrappedTarget = currentAngle;
    }

    // ------------------------------------------------------------------ public API

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
    }

    public Mode getMode() { return currentMode; }
    public void setManualAngle(double deg) { manualAngle = deg; }
    public void setManualVelocity(double dps) { manualVelocity = dps; }
    public double getAngle() { return currentAngle; }
    public double getError() { return err; }
    public double getTargetAngle() { return targetAngle; }
    public double getServoPosition() { return angleToServo(currentAngle); }
    public Storage.Alliance getAlliance() { return alliance; }
    public void setAlliance(Storage.Alliance alliance) { this.alliance = alliance; }

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

        double goal = currentAngle;
        switch (currentMode) {
            case OFF:
                if (servo.isPwmEnabled()) servo.setPwmDisable();
                return;

            case CALIBRATE:
                if (!servo.isPwmEnabled()) servo.setPwmEnable();
                currentAngle = constants.WRAP_CENTER;
                servo.setPosition(posForServo(0.5));
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
        err = goal - currentAngle;

        // move the model toward the goal at a limited speed, then command it
        double maxStep = MAX_SLEW_DPS * dt;
        currentAngle = clampToLimits(currentAngle + Range.clip(err, -maxStep, maxStep));
        servo.setPosition(angleToServo(currentAngle));
    }

    // ------------------------------------------------------------------ aiming

    /** Returns the goal angle. wrap = false is SEMI_AIM (clamped to the limits, no wrapping). */
    private double aim(boolean wrap) {
        LLResult res = limelight.getLatestResult();

        // Get a NEW target only from a new, fresh, valid result; otherwise hold the last absolute target.
        if (res.isValid() && res.getTimestamp() != lastResultTimestamp && res.getStaleness() < 100) {
            lastResultTimestamp = res.getTimestamp();
            Double tx = readTx(res, wrap);
            if (tx != null && Math.abs(tx) > AIM_DEADBAND_DEG) {
                targetAngle = currentAngle + AIM_SIGN * AIM_GAIN * tx;
            }
        }
        return wrap ? wrapToSafeRange(targetAngle) : clampToLimits(targetAngle);
    }

    /** SEMI_AIM: any tag (LLResult tx). AIMING: only the goal tags, each with its own tx, averaged. */
    private Double readTx(LLResult res, boolean onlyGoalTags) {
        if (!onlyGoalTags || alliance == null) return res.getTx();

        int[] goal = Storage.TagFamilies.get(alliance, currentLocation);
        List<LLResultTypes.FiducialResult> fids = res.getFiducialResults();
        if (fids == null) return null;

        double sum = 0;
        int n = 0;
        for (LLResultTypes.FiducialResult f : fids) {
            int id = f.getFiducialId();
            if (id == goal[0] || id == goal[3]) {
                sum += f.getTargetXDegrees();
                n++;
            }
        }
        return n > 0 ? sum / n : null;
    }

    // ------------------------------------------------------------------ geometry helpers

    /** Turret angle (deg) -> servo position (0..1). */
    private double angleToServo(double angle) {
        double pos = 0.5 + (angle - constants.WRAP_CENTER) / TURRET_RANGE_DEG.into(Degrees);
        return posForServo(pos);
    }

    private double posForServo(double pos) {
        pos = Range.clip(pos, SERVO_MIN_POS, SERVO_MAX_POS);
        return SERVO_REVERSED ? 1.0 - pos : pos;
    }

    /** Lowest reachable turret angle: servo limit, tightened by MAX_ANGLE from Storage. */
    private double minAngle() {
        double servoMin = constants.WRAP_CENTER + (SERVO_MIN_POS - 0.5) * TURRET_RANGE_DEG.into(Degrees);
        return Math.max(servoMin, constants.WRAP_CENTER - constants.MAX_ANGLE / 2.0);
    }

    private double maxAngle() {
        double servoMax = constants.WRAP_CENTER + (SERVO_MAX_POS - 0.5) * TURRET_RANGE_DEG.into(Degrees);
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