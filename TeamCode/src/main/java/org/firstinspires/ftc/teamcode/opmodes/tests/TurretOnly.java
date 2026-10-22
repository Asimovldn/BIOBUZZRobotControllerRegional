package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.pedropathing.ivy.Scheduler;

import org.firstinspires.ftc.teamcode.Context;
import org.firstinspires.ftc.teamcode.RobotOpMode;
import org.firstinspires.ftc.teamcode.systems.hardware.Turret;

import java.util.Arrays;

public class TurretOnly extends RobotOpMode {
    @Override
    public void create() {
        getTurret().reset();
        if (runtime.GlobalAlliance == null) {
            runtime.GlobalAlliance = Context.Alliance.RED;
        }
    }

    @Override
    public void setup() {
        Scheduler.reset();
        getTurret().setMode(Turret.Mode.SEMI_AIM);
        getTurret().reset();
    }

    @Override
    public void periodic() {
        telemetry.addData("Current Angle: ", getTurret().getAngle());
        telemetry.addData("Current Err: ", getTurret().getError());
        telemetry.addData("Target Angle: ", getTurret().getTargetAngle());
        telemetry.addData("Servo Pos: ", getTurret().getServoPosition());
        telemetry.addData("TA: ", getTurret().getTa());
        telemetry.addData("Location: ", getTurret().currentLocation);
        telemetry.addData("Reading Tag: ", Arrays.toString(getTurret().readingTag));

        if (Context.tagMap.get(getTurret().readingTag[0]) != null) {
            telemetry.addData("From Center: ", Context.tagMap.get(getTurret().readingTag[0]) / 2);
        }

        telemetry.update();
    }
}
