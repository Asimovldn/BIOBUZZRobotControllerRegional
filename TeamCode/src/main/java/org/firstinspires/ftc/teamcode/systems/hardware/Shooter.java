package org.firstinspires.ftc.teamcode.systems.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.systems.hardware.core.Mechanism;
import org.firstinspires.ftc.teamcode.util.PIDFController;

public class Shooter extends Mechanism {
    @Config
    public static class Tuning {
        public static double feedPower = 1;
        public static double kD = 0;
        public static double kF = 0;
        public static double kI = 0;
        public static double kP = 25;
        public static double kS = 0;
        public static double kV = 0.996;
        public static double maxOvershoot = 20;
        public static double nominalVoltage = 12;
        public static double readyTolerance = 30;
    }

    private DcMotorEx flywheel, feeder;
    private VoltageSensor battery;
    private final PIDFController pid = new PIDFController(0, 0, 0, 0);
    private double voltageScale = 1;
    private long lastVoltageNs = 0;
    private boolean spinning = false;

    @Override
    public void setup() {
        HardwareMap hw = runtime.hardwareMap;

        feeder = hw.get(DcMotorEx.class, "foreShooter");
        flywheel = hw.get(DcMotorEx.class, "backShooter");

        feeder.setDirection(DcMotorSimple.Direction.FORWARD);
        feeder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        motor = flywheel;

        for (VoltageSensor vs : hw.voltageSensor) {
            battery = vs;
            break;
        }
    }

    /** Callers pass a negative value for a forward spin. */
    @Override
    public void setTargetVelocity(double v) {
        super.setTargetVelocity(-v);
    }

    /** Positive ticks per second. */
    public void spinTo(double ticksPerSecond) {
        setTargetVelocity(-Math.abs(ticksPerSecond));
    }

    public void stopFlywheel() {
        super.setTargetVelocity(0);
    }

    @Override
    public void periodic() {
        if (flywheel == null) return;

        if (targetVelocity == 0) {
            if (spinning) {
                flywheel.setPower(0);
                pid.reset();
                spinning = false;
            }
            return;
        }

        spinning = true;

        pid.setGains(Tuning.kP, Tuning.kI, Tuning.kD, 0);
        if (Tuning.kI > 0) pid.setIntegralClamp(0.3 / Tuning.kI);

        double error = targetVelocity - flywheel.getVelocity();
        double feedforward = Tuning.kV * targetVelocity;
        double output = feedforward + pid.calculate(error);

        flywheel.setVelocity(output);
    }

    private void updateVoltage() {
        long now = System.nanoTime();
        if (battery == null || now - lastVoltageNs < 250_000_000L) return;

        lastVoltageNs = now;
        double v = battery.getVoltage();
        if (v > 5) voltageScale = Range.clip(Tuning.nominalVoltage / v, 0.8, 1.4);
    }

    /** Within readyTolerance below the target and at most maxOvershoot above it. */
    public boolean isReady() {
        if (flywheel == null || targetVelocity == 0) return false;

        double diff = Math.abs(Speed()) - Math.abs(targetVelocity);
        return diff > -Tuning.readyTolerance && diff < Tuning.maxOvershoot;
    }

    public void setFeeding(boolean on) {
        if (feeder != null) feeder.setPower(on ? Tuning.feedPower : 0);
    }

    public boolean isFeeding() {
        return feeder != null && Math.abs(feeder.getPower()) > 0.05;
    }

    /** Stops flywheel and feeder immediately. */
    public void brakeNow() {
        super.setTargetVelocity(0);
        if (flywheel != null) flywheel.setPower(0);
        if (feeder != null) feeder.setPower(0);
        pid.reset();
        spinning = false;
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
