package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Context;
import org.firstinspires.ftc.teamcode.systems.hardware.Ramp;
import org.firstinspires.ftc.teamcode.util.Util;

/** Repos servo and reset runtime */
@TeleOp(name="Reposition Servo")
public class run extends LinearOpMode {
    private double currAngle = 20;
    @Override
    public void runOpMode() throws InterruptedException {
        ServoImplEx servo = hardwareMap.get(ServoImplEx.class, "turret");

        Context ctx = Context.fromOwner(Util.Code.fromString("ReposServo"), hardwareMap);

        Ramp ramp = new Ramp();

        ramp.setup(hardwareMap);

        ramp.set(1);

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
            }

            ramp.set(0);
            ramp.set(1);

            telemetry.addData("Servo Pos: ", servo.getPosition());
            telemetry.addData("Servo Pos Ramp: ", ramp.getPosition());
            telemetry.addData("Curr Angle: ", currAngle);

            updateTelemetry(telemetry);
        }
    }
}
