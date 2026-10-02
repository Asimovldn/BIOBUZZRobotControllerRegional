package org.firstinspires.ftc.teamcode.systems.hardware;

import static dev.nextftc.units.Units.Degrees;
import static dev.nextftc.units.Units.DegreesPerSecond;

import android.hardware.Sensor;

import androidx.annotation.NonNull;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.storage.Storage;
import org.firstinspires.ftc.teamcode.util.PIDFController;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Objects;
import java.util.Set;

import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.SimpleFFCoefficients;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.hardware.sensors.NextDistanceSensor;
import dev.nextftc.hardware.webcams.NextLimelight;
import dev.nextftc.robot.Mechanism;

public class Turret implements Mechanism {

    /*
        NÃO copiar as funcionalidades dessa Torreta para ângulos que não sejam torreta.
        Isso funciona para limitar a torreta a um ângulo de atuação [-180, 180].
        Mesmo que o caminho mais curto entre 180 => 190 seja 10, ele vai escolher o -350 não bater o limite.
     */

    private final NextLimelight limelight = new NextLimelight("limelight");
    public double degs;
    private double lastWrappedTarget = 0;
    private Storage.Alliance alliance;

    private double vx, vy;

    private final Storage.TurretConstants constants = new Storage.TurretConstants();
    private NextMotor motor;
    private String currLocation;

    private final Storage.TagFamilies tags = new Storage.TagFamilies();

    // Turret heading error (+ chassis-rotation disturbance) -> target angular velocity
    private final PIDFController headingPID = new PIDFController(
            constants.posKp, constants.posKi, constants.posKd, constants.posKf);
    // posKp, posKi, posKd, posKf, edit with editConfigs("posKp", x);

    public double currentVelocity, currentAngle;
    public boolean calledPeriodic = false;
    private double err = 0, botAng = 0, lastTx = 0;

    private Mode currentMode = Mode.AIMING;

    public enum Mode { AIMING, RECENTERING, TUNE_VELOCITY, TUNE_POSITION }

    private double manualAngle = 0, manualVelocity = 0;

    public void setMode(Mode m) { currentMode = m; }
    public void setManualAngle(double deg) { manualAngle = deg; }
    public void setManualVelocity(double dps) { manualVelocity = dps; }
    public double getAngle() { return currentAngle; }

    public void applyConstants() {
        headingPID.setPIDF(constants.posKp, constants.posKi, constants.posKd, constants.posKf); // add this to your PIDFController
        motor.getVelocityConstants().setKP(constants.velKp);
        motor.getVelocityConstants().setKI(constants.velKi);
        motor.getVelocityConstants().setKD(constants.velKd);
        motor.getVelocityConstants().setKS(constants.velKs);
        motor.getVelocityConstants().setKV(constants.velKv);
        motor.getVelocityConstants().setKA(constants.velKa);
    }

    public void update() {
        currentAngle = motor.getEncoderPosition().into(Degrees);
        currentVelocity = motor.getEncoderVelocity().into(DegreesPerSecond);

        switch (currentMode) {
            case TUNE_VELOCITY:
                motor.setVelocitySetpoint(DegreesPerSecond.of(manualVelocity));
                break;

            case TUNE_POSITION:
                err = manualAngle - currentAngle;
                double v = headingPID.calculate(err, -botAng);
                v = Range.clip(v, -constants.MAX_VEL, constants.MAX_VEL);
                motor.setVelocitySetpoint(DegreesPerSecond.of(v));
                break;

            default:
                break;
        }
        motor.update();
    }

    public Turret() {
        // motor = new NextMotor(constants.motorName, Degrees.of(360.0 / 28 / constants.GEAR_RATIO));

        motor.getVelocityConstants().setKP(constants.velKp);
        motor.getVelocityConstants().setKI(constants.velKi);
        motor.getVelocityConstants().setKD(constants.velKd);
        motor.getVelocityConstants().setKS(constants.velKs);
        motor.getVelocityConstants().setKV(constants.velKv);
        motor.getVelocityConstants().setKA(constants.velKa);

        headingPID.setIntegralClamp(constants.MAX_VEL);

        limelight.startReading(0, 100);
    }

    public double getError() { return err; }
    public double getSpeed() { return currentVelocity; }
    public void botAngularVelocity(double v) { botAng = v; }

    private double __coeff(double y, double x) {
        if (x == -1) {
            return y;
        } else {
            return x;
        }

    }

    /**
    Edit Motor Velocity Coefficients, changes how much the motor reatcs to the target.
    -1 means it will stay the same.
     */
    public void editVelocityCoefficients(double kp, double ki, double kd, double ks, double ka, double kv) {
        constants.velKp = __coeff(constants.velKp, kp);
        constants.velKi = __coeff(constants.velKi, ki);
        constants.velKd = __coeff(constants.velKd, kd);
        constants.velKs = __coeff(constants.velKs, ks);
        constants.velKa = __coeff(constants.velKa, ka);
        constants.velKv = __coeff(constants.velKv, kv);
    }

    /**
     Edit Motor Velocity Coefficients, changes how much the motor reatcs to the target.
     -1 means it will stay the same.
     */
    public void editVelocityCoefficients(PIDCoefficients pid, SimpleFFCoefficients feed) {
        editVelocityCoefficients(pid.kP, pid.kI, pid.kD, feed.kS, feed.kA, feed.kV);
    }

    /**
     Edit Position Coefficients, changes how much the turret reacts to the heading error.
     -1 means it will stay the same.
     */
    public void editPositionCoefficients(double kp, double ki, double kd, double kv) {
        constants.posKp = __coeff(constants.velKp, kp);
        constants.posKi = __coeff(constants.velKi, ki);
        constants.posKd = __coeff(constants.velKd, kd);
        constants.posKf = __coeff(constants.velKv, kv);
    }

    /**
     Edit Position Coefficients, changes how much the turret reacts to the heading error.
     -1 means it will stay the same.
     */
    public void editPositionCoefficients(PIDFCoefficients coeff) {
        editPositionCoefficients(coeff.p, coeff.i, coeff.d, coeff.f);
    }


    /**
     * Wraps {@code angle} into the turret's safe mechanical range around WRAP_CENTER
     * {@code (+-MAX_ANGLE/2)}, at the same dir as last target so value at the limit
     * doesn't turn the turret back and forth by 350 every cycle.
     */
    private double wrapToSafeRange(double angle) {
        double half = constants.MAX_ANGLE / 2.0;
        double center = constants.WRAP_CENTER;

        // Normalização Simples
        double wrapped = angle;
        while (wrapped > center + half) wrapped -= 360;
        while (wrapped <= center - half) wrapped += 360;

        // Simple check so that if the target is close to the max_angle, it does not spin 360 every single cycle (that would be sad)
        double alt = wrapped + (wrapped < lastWrappedTarget ? 360 : -360);
        boolean inRange = alt > center - half && alt <= center + half;
        boolean closerToLast = Math.abs(alt - lastWrappedTarget) + constants.WRAP_HYSTERESIS
                < Math.abs(wrapped - lastWrappedTarget);
        if (inRange && closerToLast) wrapped = alt;

        lastWrappedTarget = wrapped;

        return wrapped;
    }

    public Command changeLocation() {
        return instant(() -> {
            currLocation = (Objects.equals(currLocation, "Audience")) ? "Scoring" : "Audience";
        }).setPriority(100);
    }

    public double getRequiredAngle(double pollenVelocity, double err, double robotX, double robotY) {
        return Math.atan2(pollenVelocity * Math.sin(err) - vy, pollenVelocity * Math.cos(err) - vx);
    }

    /** Check if all tags are seen, if not, this means that the CELL bar is blocking
     * one of the tags or the robot is completely unaimed. If so, aim for the border of the
     * aprilTags for better condition.
     * @param seen current seen tags.
     * @param currTags current target tags;
     * @return boolean, prioritize middle.
     */
    private boolean attemptToPrioritizeMiddle(LLResult seen, int[] currTags) {
        Set<Integer> tags = Set.of(currTags[0], currTags[1], currTags[2], currTags[3]);

        if (!seen.isValid()) return true;
        int inCurrTags = 0;
        for (LLResultTypes.FiducialResult fid : seen.getFiducialResults()) {
            if (tags.contains(fid.getFiducialId())) {
                inCurrTags++;
            }
        }

        return inCurrTags == currTags.length;
    }

    /**
     * Attempts to guess the side to prioritize given the seen tags.
     * If there are more tags in one side, aim for the other.
     * @param seen
     * @param currTags
     * @param sidePriorityTag
     * @return
     */
    private double sidePriority(LLResult seen, int[] currTags, int sidePriorityTag) {
        Set<Integer> rightSide = Set.of(currTags[3], currTags[4]);
        Set<Integer> leftSide = Set.of(currTags[0], currTags[1]);

        int side = rightSide.contains(sidePriorityTag) ? 1 : -1;
        // RIGHT (1), LEFT (-1)



        ArrayList<Integer> seenTags = new ArrayList<>();

        for (LLResultTypes.FiducialResult fid : seen.getFiducialResults()) {
            seenTags.add(fid.getFiducialId());
        }

        long l = seenTags.stream()
                .filter(leftSide::contains)
                .count();

        long r = seenTags.stream()
                .filter(rightSide::contains)
                .count();

        if (l > r && side == -1) {
            return 0.5;
        } else if (l > r && side == 1) {
            return 1.5;
        } else if (r > l && side == -1) {
            return 1.5;
        } else if (r > l && side == 1) {
            return 0.5;
        } else {
            return 1;
        }
    }

    private void passInRobotVelocity(double vx, double vy) {
        this.vx = vx; this.vy = vy;
    }

    private Command aim() {
        return infinite(() -> {
            LLResult res = limelight.getLatestResult();
            boolean found = false;

            int[] currTags = tags.get(alliance, currLocation);
            int count = 0;


            if (attemptToPrioritizeMiddle(res, currTags)) {
                for (LLResultTypes.FiducialResult fid : res.getFiducialResults()) {
                    if (fid.getFiducialId() != currTags[0] && fid.getFiducialId() != currTags[3])
                        continue;
                    err += res.getTx();
                    found = true;
                    count++;
                }
            } else {
                for (LLResultTypes.FiducialResult fid : res.getFiducialResults()) {
                    if (fid.getFiducialId() != currTags[0] && fid.getFiducialId() != currTags[3])
                        continue;
                    err += res.getTx() * sidePriority(res, currTags, fid.getFiducialId());
                    found = true;
                    count++;
                }
            }


            if (count > 0) err /= count;

            if (!found || !res.isValid()) err = lastTx;

            calledPeriodic = true;
            currentAngle = motor.getEncoderPosition().into(Degrees);

            double desiredAngle = currentAngle + err;
            double wrappedTarget = wrapToSafeRange(desiredAngle);

            double angleError = wrappedTarget - currentAngle;

            double targetAngularVelocity = headingPID.calculate(angleError, -botAng);
            targetAngularVelocity = Range.clip(targetAngularVelocity, -constants.MAX_VEL, constants.MAX_VEL);

            motor.setVelocitySetpoint(DegreesPerSecond.of(targetAngularVelocity));

            currentVelocity = motor.getEncoderVelocity().into(DegreesPerSecond);
            motor.update();

            lastTx = err;
        });
    }

    public Storage.Alliance getAlliance() {
        return alliance;
    }

    public void setAlliance(Storage.Alliance alliance) {
        this.alliance = alliance;
    }

    /**
     * editCoefficients("velKp", 12)
     * @param k name of the gain as a String, utilize it EXACTLY as mentioned in Storage.
     * @param x new value.
     * @return Command to be scheduled.
     */
    private Command editCoefficient(String k, double x) {
        return instant(() -> {
            constants.editSetting(k, x);
        });
    }

    @Override
    public void periodic() {
        switch (currentMode) {
            case AIMING: aim().execute(); break;
        }
    }
}