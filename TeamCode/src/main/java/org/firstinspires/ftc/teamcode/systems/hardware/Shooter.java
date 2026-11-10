package org.firstinspires.ftc.teamcode.systems.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.systems.hardware.core.Mechanism;
import org.firstinspires.ftc.teamcode.util.PIDFController;

public class Shooter extends Mechanism {
    /** Fixed launch angle of the hood, degrees above horizontal. */
    public static final double HOOD_DEG = 70;

    @Config
    public static class Tuning {
        public static double feedPower = 0.97;

        public static double kP = 25;
        public static double kI = 0;
        public static double kD = 0;
        public static double kV = 0.71;

        public static double hubP = 15;
        public static double hubI = 0;
        public static double hubD = 0;
        public static double hubF = 30;

        public static boolean voltageComp = false;
        public static boolean allowReverse = false;
        public static double nominalVoltage = 12;

        public static double readyTolerance = 30;
        public static double readyHysteresis = 15;
        public static double maxOvershoot = 20;
        public static double readyHoldMs = 30;

        public static double dipTicks = 50;
        public static double rearmTicks = 20;
        public static double shotGraceMs = 300;
        public static double targetSettleMs = 250;
    }

    /** Single loop on motor power: power = kS + kV * target + PID, scaled by battery voltage. Run ShooterTune to get kS and kV. */
    @Config
    public static class PowerTuning {
        public static boolean enabled = true;
        public static double kD = 0;
        public static double kI = 0;
        public static double kP = 0.032;
        public static double kS = 0;
        public static double kV = 0.00052;
        public static double maxPower = 1;
        public static boolean voltageComp = true;
    }

    private DcMotorEx flywheel, feeder;
    private VoltageSensor battery;
    private final PIDFController pid = new PIDFController(0, 0, 0, 0);

    private double velocity;
    private double voltage = 12, voltageScale = 1;
    private double openLoopPower = Double.NaN;
    private double lastTarget;
    private double appliedP = Double.NaN, appliedI, appliedD, appliedF;
    private long lastVoltageNs, lastTargetChangeNs, lastFeedNs, lastShotNs, readySinceNs;
    private boolean spinning, ready, armed, feedOn, powerMode, modeKnown;
    private int shots;

    @Override
    public void setup() {
        HardwareMap hw = runtime.hardwareMap;

        feeder = hw.get(DcMotorEx.class, "foreShooter");
        flywheel = hw.get(DcMotorEx.class, "backShooter");

        feeder.setDirection(DcMotorSimple.Direction.FORWARD);
        feeder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        motor = flywheel;

        for (VoltageSensor vs : hw.voltageSensor) {
            battery = vs;
            break;
        }

        syncHardware();
    }

    /** Legacy sign: callers pass a negative value for a forward spin. Prefer {@link #spinTo(double)}. */
    @Override
    public void setTargetVelocity(double v) {
        super.setTargetVelocity(-v);
    }

    /** Positive ticks per second. */
    public void spinTo(double ticksPerSecond) {
        super.setTargetVelocity(Math.abs(ticksPerSecond));
    }

    public void stopFlywheel() {
        super.setTargetVelocity(0);
    }

    /** Bypasses the controller, used by ShooterTune to characterize kS and kV. Pass null to release. */
    public void setOpenLoopPower(Double power) {
        openLoopPower = power == null ? Double.NaN : power;
    }

    @Override
    public void periodic() {
        if (flywheel == null) return;

        long now = System.nanoTime();
        velocity = flywheel.getVelocity();

        updateVoltage(now);
        syncHardware();

        double target = Math.abs(targetVelocity);
        if (Math.abs(target - lastTarget) > 15) lastTargetChangeNs = now;
        lastTarget = target;

        updateReady(now, target);

        if (!Double.isNaN(openLoopPower)) {
            spinning = true;
            flywheel.setPower(Range.clip(openLoopPower, 0, 1));
            return;
        }

        if (target == 0) {
            if (spinning) {
                flywheel.setPower(0);
                pid.reset();
                spinning = false;
            }
            return;
        }

        spinning = true;
        double error = target - velocity;

        if (powerMode) {
            pid.setGains(PowerTuning.kP, PowerTuning.kI, PowerTuning.kD, 0);
            if (PowerTuning.kI > 0) pid.setIntegralClamp(0.3 / PowerTuning.kI);

            double power = PowerTuning.kS + PowerTuning.kV * target + pid.calculate(error);
            if (PowerTuning.voltageComp) power *= voltageScale;

            flywheel.setPower(Range.clip(power, 0, PowerTuning.maxPower));
            return;
        }

        pid.setGains(Tuning.kP, Tuning.kI, Tuning.kD, 0);
        if (Tuning.kI > 0) pid.setIntegralClamp(0.3 / Tuning.kI);

        double feedforward = Tuning.kV * target * (Tuning.voltageComp ? voltageScale : 1);
        double output = feedforward + pid.calculate(error);

        flywheel.setVelocity(Tuning.allowReverse ? output : Math.max(0, output));
    }

    private void syncHardware() {
        boolean wantPower = PowerTuning.enabled || !Double.isNaN(openLoopPower);

        if (!modeKnown || wantPower != powerMode) {
            flywheel.setMode(wantPower ? DcMotor.RunMode.RUN_WITHOUT_ENCODER : DcMotor.RunMode.RUN_USING_ENCODER);
            powerMode = wantPower;
            modeKnown = true;
            pid.reset();
        }

        if (powerMode) return;

        if (appliedP != Tuning.hubP || appliedI != Tuning.hubI || appliedD != Tuning.hubD || appliedF != Tuning.hubF) {
            flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(Tuning.hubP, Tuning.hubI, Tuning.hubD, Tuning.hubF));
            appliedP = Tuning.hubP;
            appliedI = Tuning.hubI;
            appliedD = Tuning.hubD;
            appliedF = Tuning.hubF;
        }
    }

    private void updateVoltage(long now) {
        if (battery == null || now - lastVoltageNs < 250_000_000L) return;

        lastVoltageNs = now;
        double v = battery.getVoltage();
        if (v > 5) {
            voltage = v;
            voltageScale = Range.clip(Tuning.nominalVoltage / v, 0.8, 1.4);
        }
    }

    private void updateReady(long now, double target) {
        if (target == 0) {
            ready = false;
            armed = false;
            readySinceNs = 0;
            return;
        }

        double speed = Math.abs(velocity);
        double diff = speed - target;
        double widen = ready ? Tuning.readyHysteresis : 0;
        boolean inBand = diff > -(Tuning.readyTolerance + widen) && diff < Tuning.maxOvershoot + widen;

        if (!inBand) {
            ready = false;
            readySinceNs = 0;
        } else if (!ready) {
            if (readySinceNs == 0) readySinceNs = now;
            ready = (now - readySinceNs) * 1e-6 >= Tuning.readyHoldMs;
        }

        detectShot(now, target - speed);
    }

    private void detectShot(long now, double below) {
        if ((now - lastTargetChangeNs) * 1e-6 < Tuning.targetSettleMs) {
            armed = false;
            return;
        }

        if (!armed) {
            if (below < Tuning.rearmTicks) armed = true;
            return;
        }

        boolean feeding = feedOn || (now - lastFeedNs) * 1e-6 < Tuning.shotGraceMs;
        if (feeding && below > Tuning.dipTicks) {
            shots++;
            lastShotNs = now;
            armed = false;
        }
    }

    /** Cached once per loop, signed as the encoder reports it. */
    @Override
    public double Speed() {
        return velocity;
    }

    public double speed() {
        return Math.abs(velocity);
    }

    public double getVoltage() {
        return voltage;
    }

    /** Debounced, with hysteresis: within readyTolerance below the target and at most maxOvershoot above it. */
    public boolean isReady() {
        return flywheel != null && targetVelocity != 0 && ready;
    }

    /** Balls detected leaving since the last reset, from the RPM dip while feeding. */
    public int shotCount() {
        return shots;
    }

    public void resetShotCount() {
        shots = 0;
        lastShotNs = 0;
    }

    public double msSinceLastShot() {
        return lastShotNs == 0 ? Double.MAX_VALUE : (System.nanoTime() - lastShotNs) * 1e-6;
    }

    public void setFeeding(boolean on) {
        if (feeder == null) return;
        if (on) lastFeedNs = System.nanoTime();
        if (on == feedOn) return;

        feedOn = on;
        feeder.setPower(on ? Tuning.feedPower : 0);
    }

    public boolean isFeeding() {
        return feedOn;
    }

    /** Stops flywheel and feeder immediately. */
    public void brakeNow() {
        super.setTargetVelocity(0);
        openLoopPower = Double.NaN;

        if (flywheel != null) flywheel.setPower(0);
        if (feeder != null) feeder.setPower(0);

        feedOn = false;
        ready = false;
        armed = false;
        spinning = false;
        pid.reset();
    }

    /** Feeds only while the flywheel is ready. Never finishes by itself, cancel it or use {@link #setFeeding(boolean)}. */
    public Command enableLaunch() {
        return Command.build()
                .setExecute(() -> setFeeding(isReady()))
                .requiring(this);
    }

    public Command disableLaunch() {
        return Command.build()
                .setExecute(() -> setFeeding(false))
                .setDone(() -> !isFeeding());
    }

    /** The brake runs when the command runs, not when it is built. */
    public Command fullBrake() {
        return Command.build()
                .setStart(this::brakeNow)
                .setExecute(this::brakeNow)
                .setDone(() -> true);
    }
}
