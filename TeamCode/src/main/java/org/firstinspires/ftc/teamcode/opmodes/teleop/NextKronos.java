package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.storage.Storage;
import org.firstinspires.ftc.teamcode.systems.software.Camera;

import dev.nextftc.control.feedback.PIDController;

@TeleOp
public class NextKronos extends OpMode {
    private Follower follower;
    private final PoseFactory p = PoseFactory.degrees();
    private double targetBoost = 0; // TODO: CHANGE THIS LATER

    private double targetHeading = 0;

    private String currentAlliance = "Blue";
    private String currentSide = "Audience";

    private PIDController headingPID = new PIDController(0.3, 0.01, 0.1, true);

    private final Pose startPose = p.of(24, 24, 0);
    private boolean isholding;
    private Pose toBeHeld;

    private Controller servoControl;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);

        if (!Storage.autonomousEnd.equals(new Pose(0, 0, 0))) {
            follower.setPose(Storage.autonomousEnd);
        } else {
            follower.setPose(startPose);
        }

        if (Storage.targetBoost != 0) {
            targetBoost = Storage.targetBoost;
        }

        servoControl = Controller.pid(0, 0, 0);
        servoControl.calculate(0, 0);
    }

    @Override
    public void start() {
        int[] tags = Storage.TagFamilies.get(Storage.Alliance.valueOf(currentAlliance), currentSide);

        /*
        double v = new Camera(hardwareMap).getX(tags[0]);
        double pwr = compress(v);
        */

        // servo.setPower(pwr * kP, pwr - lastPwr * kD);

        servoControl.reset();

        targetHeading = follower.pose().heading();
    }

    @Override
    public void loop() {
        double speedDamp = 1 - gamepad1.left_trigger;
        double turn = -gamepad1.right_stick_x;

        ManualDrive.driveOrHold(follower,
                -gamepad1.left_stick_y * speedDamp,
                -gamepad1.left_stick_x * speedDamp,
                turn * speedDamp
        );

        follower.update();
    }
}
