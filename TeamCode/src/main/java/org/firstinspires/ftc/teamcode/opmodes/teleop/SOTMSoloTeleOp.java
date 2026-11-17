package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.util.ResetPoses;

/**
 * One gamepad. Sticks drive. LT hold or A toggle: spin. RT: feed. RB / LB: intake in / out. B: SOTM on/off.
 * X: ramp toggle. Y: ramp max up. D-pad up/down: manual velocity. D-pad left: reset turret. D-pad right: log shot.
 * Start: manual-only velocity. Hold Back for resets: A/B/X/Y presets, D-pad walls (add LB for front against wall).
 */
@TeleOp(name = "SOTM Solo Red", group = "TeleOp")
public class SOTMSoloTeleOp extends SOTMTeleOp {
    private boolean spinLatch;

    @Override
    protected Controls controls() {
        Controls c = new Controls();
        boolean modifier = gamepad1.back;

        c.feed = gamepad1.right_trigger > 0.5;
        c.intake = c.feed || gamepad1.right_bumper ? 1 : gamepad1.left_bumper ? -1 : 0;
        c.toggleManual = gamepad1.startWasPressed();

        if (!modifier) {
            if (gamepad1.aWasPressed()) spinLatch = !spinLatch;
            c.toggleSotm = gamepad1.bWasPressed();
            c.rampAlter = gamepad1.xWasPressed();
            c.rampMax = gamepad1.yWasPressed();
            c.velUp = gamepad1.dpadUpWasPressed();
            c.velDown = gamepad1.dpadDownWasPressed();
            c.resetTurret = gamepad1.dpadLeftWasPressed();
            c.logShot = gamepad1.dpadRightWasPressed();
        }

        c.spin = spinLatch || gamepad1.left_trigger > 0.5;
        return c;
    }

    @Override
    protected Gamepad rumblePad() {
        return gamepad1;
    }

    @Override
    protected void handleResets() {
        if (!gamepad1.back) return;

        Pose now = getFollower().pose();

        if (gamepad1.aWasPressed()) resetPose(forAlliance(ResetPoses.AUDIENCE_START));
        if (gamepad1.bWasPressed()) resetPose(forAlliance(ResetPoses.SCORING_CORNER));
        if (gamepad1.xWasPressed()) resetPose(forAlliance(ResetPoses.GARDEN));
        if (gamepad1.yWasPressed()) resetPose(forAlliance(ResetPoses.SCORING_FLOWER));

        boolean front = gamepad1.left_bumper;
        if (gamepad1.dpadDownWasPressed()) resetPose(wallPose(now, ResetPoses.Wall.AUDIENCE, front));
        if (gamepad1.dpadUpWasPressed()) resetPose(wallPose(now, ResetPoses.Wall.SCORING, front));
        if (gamepad1.dpadLeftWasPressed()) resetPose(wallPose(now, ResetPoses.Wall.LEFT, front));
        if (gamepad1.dpadRightWasPressed()) resetPose(wallPose(now, ResetPoses.Wall.RIGHT, front));
    }
}
