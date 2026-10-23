package org.firstinspires.ftc.teamcode.systems.hardware;

import static com.pedropathing.ivy.groups.Groups.parallel;

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
        public static double kP = 0.01, kI = 0, kD = 0.002;
        public static double kV = 0.00036, kS = 0, kF = 0;
        public static double readyTolerance = 70;
        public static double feedPower = -0.9;
        public static double nominalVoltage = 12.5;
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
        updateVoltage();

        pid.setGains(Tuning.kP, Tuning.kI, Tuning.kD, Tuning.kF);
        if (Tuning.kI > 0) pid.setIntegralClamp(0.3 / Tuning.kI);

        double error = targetVelocity - flywheel.getVelocity();
        double output = Tuning.kV * targetVelocity
                + Tuning.kS * Math.signum(targetVelocity)
                + pid.calculate(error);

        flywheel.setPower(Range.clip(output * voltageScale, -1, 1));
    }

    private void updateVoltage() {
        long now = System.nanoTime();
        if (battery == null || now - lastVoltageNs < 250_000_000L) return; // Runs around 4 times per second

        lastVoltageNs = now;
        double v = battery.getVoltage();
        if (v > 5) voltageScale = Range.clip(Tuning.nominalVoltage / v, 0.8, 1.4);
    }

    public boolean isReady() {
        return targetVelocity != 0 && atSpeed(targetVelocity, Tuning.readyTolerance);
    }

    public Command enableLaunch() {
        return Command.build()
                .setExecute(() -> feeder.setPower(Tuning.feedPower))
                .requiring(this);
    }

    public Command disableLaunch() {
        return Command.build()
                .setExecute(() -> feeder.setPower(0))
                .setDone(() -> Math.abs(feeder.getPower()) < 0.1);
    }

    public Command fullBrake() {
        setTargetVelocity(0);
        return parallel(disableLaunch());
    }
}
