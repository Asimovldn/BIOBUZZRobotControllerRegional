package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotOpMode;
import org.firstinspires.ftc.teamcode.systems.hardware.Turret;

@TeleOp(name = "Turret Calibrate", group = "Test")
public class TurretCalibrate extends RobotOpMode {
    private double command = 0;

    @Override
    public void setup() {
        Scheduler.reset();
        getTurret().setMode(Turret.Mode.TUNE_POSITION);
    }

    @Override
    public void periodic() {
        if (gamepad1.aWasPressed()) command = 0;
        if (gamepad1.bWasPressed()) command = 60;
        if (gamepad1.xWasPressed()) command = -60;
        if (gamepad1.yWasPressed()) command = 90;
        if (gamepad1.dpadUpWasPressed()) command += 5;
        if (gamepad1.dpadDownWasPressed()) command -= 5;
        if (gamepad1.dpadRightWasPressed()) command += 1;
        if (gamepad1.dpadLeftWasPressed()) command -= 1;

        getTurret().setManualAngle(command);

        double pos = getTurret().getServoPosition();

        telemetry.addData("Command (deg)", command);
        telemetry.addData("Model angle", getTurret().getAngle());
        telemetry.addData("Servo pos", pos);
        telemetry.addData("Pulse (us)", 500 + pos * 2000);
        telemetry.addData("turretPerServo", Turret.TurretTune.turretPerServo);
        telemetry.addData("servoRangeDeg", Turret.TurretTune.servoRangeDeg);
        telemetry.addData("centerTrimDeg", Turret.TurretTune.centerTrimDeg);
        telemetry.addLine("A=0  B=+60  X=-60  Y=+90  dpad = +-5 / +-1");
        telemetry.addLine("Measure the real angle P at command A: new turretPerServo = old * P / A");
        telemetry.addLine("At command 0 the turret should face forward, else adjust centerTrimDeg");
        telemetry.update();
    }
}
