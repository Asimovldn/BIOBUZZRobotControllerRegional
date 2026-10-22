package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;

import org.jetbrains.annotations.Nullable;

public class LazyDrive {
    @Nullable
    private Double headingTarget;

    private static final double INPUT_THRESHOLD = 0.03;
    private static final double TURN_THRESHOLD = 0.04;

    private static final double HEADING_KP = 1.5;
    
    private DrivePowers pwrs;

    /**
     * Auto lock heading + hold position, can be used with fieldCentric.
     * Pass in the fieldCentric powers inside {@code powers}.
     * @param follower the follower
     * @param powers the powers to the motors
     */
    public void drive(Follower follower, DrivePowers powers) {
        boolean inputActive = hasInput(pwrs, INPUT_THRESHOLD);

        pwrs = powers;

        /*
            if ((follower.manual() || follower.idle())
                    && !inputActive
                    && follower.velocity().toVector2D().magnitude() < 0.2) {
                headingTarget = null;
                follower.hold(follower.pose());
                return;
            }
         */

        if (pwrs == null) return;

        boolean translating =
                Math.abs(pwrs.forward()) > INPUT_THRESHOLD
                        || Math.abs(pwrs.strafe()) > INPUT_THRESHOLD;

        boolean turning = Math.abs(pwrs.turn()) > TURN_THRESHOLD;

        if (turning) {
            headingTarget = null;
        } else if (translating) {
            double currentHeading = follower.pose().heading();

            if (headingTarget == null) {
                headingTarget = currentHeading;
            }

            double error = Math.atan2(
                    Math.sin(headingTarget - currentHeading),
                    Math.cos(headingTarget - currentHeading)
            );

            double correction = HEADING_KP * error;

            // Preserve translation while correcting heading.

            pwrs = new DrivePowers(
                    pwrs.forward(),
                    pwrs.strafe(),
                    correction
            );
        } else {
            headingTarget = null;
        }

        follower.manual(pwrs);
    }

    /**
     * Drive with Heading Lock + Hold Position.
     * @param follower the follower
     * @param forward strength forward, auto locks heading
     * @param strafe strength to the side, auto locks heading
     * @param heading turn strength.
     */
    public void drive(Follower follower, double forward, double strafe, double heading) {
        drive(follower, new DrivePowers(forward, strafe, heading));
    }

    /**
     * Just a normal drive.
     */
    public void lazy(Follower follower, double forward, double strafe, double turn) {
        follower.manual(forward, strafe, turn);
    }

    /** Exactly like {@code drive}, but fieldCentric. */
    public void fieldCentric(Follower follower, double forward, double strafe, double turn) {
        drive(follower, ManualDrive.fieldCentric(forward, strafe, turn, follower.pose().heading()));
    }

    public static boolean hasInput(DrivePowers pwrs, double threshold) {
        if (pwrs == null) return false;
        return Math.abs(pwrs.forward()) > threshold
                || Math.abs(pwrs.strafe()) > threshold
                || Math.abs(pwrs.turn()) > threshold;
    }
    
    public boolean isDriving() {
        return LazyDrive.hasInput(pwrs, INPUT_THRESHOLD);
    }

    public DrivePowers get() {
        return pwrs;
    }
}
