package org.firstinspires.ftc.teamcode.opmodes.tests;


import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Context;
import org.firstinspires.ftc.teamcode.RobotOpMode;
import org.firstinspires.ftc.teamcode.systems.hardware.Turret;

import java.util.Arrays;

@TeleOp
public class RedCalibrate extends RobotOpMode {
    public double strength = 15;

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
        getTurret().setMode(Turret.Mode.AIMING);
        getTurret().reset();
    }

    @Override
    public void periodic() {
        telemetry.addData("Current Velocity: ", getIntake().Speed());
        telemetry.addData("Current Angle: ", getTurret().getAngle());
        telemetry.addData("Current Err: ", getTurret().getError());
        telemetry.addData("Target Angle: ", getTurret().getTargetAngle());
        telemetry.addData("Servo Pos: ", getTurret().getServoPosition());
        telemetry.addData("TA: ", getTurret().getTa());
        telemetry.addData("Location: ", getTurret().currentLocation);
        telemetry.addData("Reading Tag: ", Arrays.toString(getTurret().readingTag));

        // 12000deg/s (1666rpm) muito fraco
        // 25000deg/s bateu no TETO

        // 8500 config (3100rpm REAL) -> acertou por pouco
        // 8600 config (295b0rpm REAL) -> correto!!

        // 8600 * 0.346 -> ~3000.
        // 8380 * 0.346 -> ~2900.

        // INTAKE VELOCITY IS SUPPOSED TO BE [8600].
        // MOTOR REPORTS [3000]RPM, NOT COUNTING FOR (5:1) REDUCTION.
        // UTILIZE THIS VALUE FOR PERFORMANCE.


        getIntake().setThrottle(gamepad2.left_stick_y);

        if (gamepad2.dpadDownWasPressed()) {
            strength -= 0.5;
        } else if (gamepad2.dpadUpWasPressed()) {
            strength += 0.5;
        }

        telemetry.addData("Strength: ", strength);

        double target = 100 * strength;

        telemetry.addData("Target (ticks/s): ", target * gamepad2.left_trigger);
        telemetry.addData("Flywheel (ticks/s): ", getShooter().Speed());

        if (Math.abs(strength) > 0) {
            getShooter().setTargetVelocity(target * gamepad2.left_trigger);
        } else {
            getShooter().fullBrake().execute();
            getTurret().editGain(0.4);
        }

        if (gamepad2.right_bumper && getTurret().isOnTarget(1.5)) {
            Command c = getShooter().enableLaunch();
            c.execute();
        } else {
            getShooter().disableLaunch().execute();
        }

        if (getShooter().isReady() && Math.abs(target) > 200) {
            gamepad2.rumble(100);
        }

        if (gamepad2.yWasPressed()) {
            getTurret().reset();

            gamepad2.rumbleBlips(2);
        }

        if (gamepad2.bWasPressed()) {
            getTurret().reset();

            runtime.switchLocation();
            gamepad2.rumbleBlips(3);
        }

        drive(gamepad1);

        telemetry.update();
    }
}
