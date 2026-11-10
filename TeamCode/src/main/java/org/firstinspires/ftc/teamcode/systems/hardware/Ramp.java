package org.firstinspires.ftc.teamcode.systems.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.systems.hardware.core.Mechanism;

public class Ramp extends Mechanism {
    public Servo servo;

    @Override
    public void setup() {
        setup(runtime.hardwareMap);
    }

    public void setup(HardwareMap hw) {
        flag = true;
        servo = hw.get(Servo.class, "flower");
        servo.setPosition(off());
    }

    public double getPosition() {
        return servo.getPosition();
    }

    @Config
    public static class FlowerTune {
        public static double alpha = 0.15;
    }

    public double MIN_ANGLE = 20, MAX_ANGLE = 40;
    public boolean flag = false;

    /** Returns [left, right] */
    public double angleToServo(double ang) {
        double angle = Range.clip(ang, MIN_ANGLE, MAX_ANGLE);

        return 1 + (((angle - MIN_ANGLE) * (FlowerTune.alpha - 1)) / (MAX_ANGLE - MIN_ANGLE));
    }

    public double off() {
        return 0.53;
    }

    public double on() {
        return FlowerTune.alpha;
    }

    public double extend() {
        return 0.84;
    }

    public void alter() {
        if (flag) {
            servo.setPosition(off());
        } else {
            servo.setPosition(on());
        }

        flag = !flag;
    }

    public boolean getFlag() { return flag; }

    public void maxUp() {
        servo.setPosition(extend());
        flag = false;
    }

    public void set(double p) {
        if (p > 0.5) {
            flag = false;
        } else {
            flag = true;
        }
        servo.setPosition(p);
    }

    public void setServoToPoses(double pos) {
        servo.setPosition(pos);
    }
}
