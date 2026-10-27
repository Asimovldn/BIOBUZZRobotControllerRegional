package org.firstinspires.ftc.teamcode.systems.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.systems.hardware.core.Mechanism;
import org.firstinspires.ftc.teamcode.util.Util;

public class Hood extends Mechanism {
    public Servo servo;

    @Override
    public void setup() {
        servo = runtime.hardwareMap.get(Servo.class, "servo");
        servo.setPosition(1);
    }

    public void setup(HardwareMap hw) {
        servo = hw.get(Servo.class, "servo");
        servo.setPosition(1);
    }

    public double getPosition() {
        return servo.getPosition();
    }

    public double MIN_ANGLE = 20, MAX_ANGLE = 40;

    /** Returns [left, right] */
    public double angleToServo(double ang) {
        double angle = Range.clip(ang, MIN_ANGLE, MAX_ANGLE);

        return 1 + (((angle - MIN_ANGLE) * (0.65 - 1)) / (40 - 20));
    }

    public void setServoToPoses(double pos) {
        servo.setPosition(pos);
    }
}
