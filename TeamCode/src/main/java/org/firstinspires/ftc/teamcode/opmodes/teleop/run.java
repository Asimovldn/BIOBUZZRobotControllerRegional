package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Context;
import org.firstinspires.ftc.teamcode.systems.hardware.Hood;
import org.firstinspires.ftc.teamcode.util.Util;

/** Repos servo and reset runtime */
@TeleOp(name="Reposition Servo")
public class run extends LinearOpMode {
    private double currAngle = 20;
    @Override
    public void runOpMode() throws InterruptedException {
        ServoImplEx servo = hardwareMap.get(ServoImplEx.class, "turret");

        Context ctx = Context.fromOwner(Util.Code.fromString("ReposServo"), hardwareMap);

        Hood hood = new Hood();

        hood.setup(hardwareMap);

        waitForStart();

        servo.setPwmEnable();
        servo.setPosition(0.5);

        while (opModeIsActive()) {
            if (!gamepad2.left_bumper) {
                if (gamepad2.dpadUpWasPressed() || gamepad1.dpadUpWasPressed()) {
                    servo.setPosition(0.5);
                } else if (gamepad2.dpadLeftWasPressed() || gamepad1.dpadLeftWasPressed()) {
                    servo.setPosition(0);
                } else if (gamepad2.dpadRightWasPressed() || gamepad1.dpadRightWasPressed()) {
                    servo.setPosition(1);
                }
            } else {
                hood.setServoToPoses(hood.angleToServo(currAngle));

                if (gamepad2.dpadUpWasPressed()) currAngle++;
                if (gamepad2.dpadDownWasPressed()) currAngle--;
            }

            telemetry.addData("Servo Pos: ", servo.getPosition());
            telemetry.addData("Servo Pos Hood: ", hood.getPosition());
            telemetry.addData("Curr Angle: ", currAngle);

            updateTelemetry(telemetry);
        }
    }
}
