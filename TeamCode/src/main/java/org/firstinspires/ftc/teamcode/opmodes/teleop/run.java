package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp
public class run extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ServoImplEx servo = hardwareMap.get(ServoImplEx.class, "turret");

        waitForStart();

        servo.setPwmEnable();
        servo.setPosition(0.5);

        while (opModeIsActive()) {
            servo.setPosition(0.5);
            telemetry.addData("Servo Pos: ", servo.getPosition());
            updateTelemetry(telemetry);
        }
    }
}
