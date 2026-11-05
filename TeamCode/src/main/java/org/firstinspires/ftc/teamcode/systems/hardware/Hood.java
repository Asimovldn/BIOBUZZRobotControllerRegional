package org.firstinspires.ftc.teamcode.systems.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.systems.hardware.core.Mechanism;
import org.firstinspires.ftc.teamcode.util.Util;

public class Hood extends Mechanism {
    public Servo servo;

    @Override
    public void setup() {
        servo = runtime.hardwareMap.get(Servo.class, "flower");
        servo.setPosition(1);
    }

    public void setup(HardwareMap hw) {
        servo = hw.get(Servo.class, "flower");
        servo.setPosition(1);
    }

    public double getPosition() {
        return servo.getPosition();
    }

    @Config
    public static class FlowerTune {
        public static double alpha = 0.65;
    }

    public double MIN_ANGLE = 20, MAX_ANGLE = 40;
    public boolean flag = false;

    /** Returns [left, right] */
    public double angleToServo(double ang) {
        double angle = Range.clip(ang, MIN_ANGLE, MAX_ANGLE);

        return 1 + (((angle - MIN_ANGLE) * (FlowerTune.alpha - 1)) / (MAX_ANGLE - MIN_ANGLE));
    }

    public double on() {
        return angleToServo(MAX_ANGLE);
    }

    public double off() {
        return angleToServo(MIN_ANGLE);
    }

    public void alter() {
        if (flag) {
            servo.setPosition(off());
        } else {
            servo.setPosition(on());
        }

        flag = !flag;
    }

    public void set(double p) {
        servo.setPosition(p);
    }

    public void setServoToPoses(double pos) {
        servo.setPosition(pos);
    }
}
