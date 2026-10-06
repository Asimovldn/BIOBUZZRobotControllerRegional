package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.jetbrains.annotations.Nullable;

// TODO: Aparentemente o yPod está sempre invertido?? (conferir cabos).
public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("leftFront");
        c.frontRightName.set("rightFront");
        c.backLeftName.set("leftBack");
        c.backRightName.set("rightBack");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-2.415661549004983);
        c.yPodOffset.set(-2.7663103238804134);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.17803273467284184);
                Controller secondaryTranslationalForward = Controller.proportional(0.06577833833933143);
                Controller primaryTranslationalLateral = Controller.proportional(0.4337474964591333);
                Controller secondaryTranslationalLateral = Controller.proportional(0.16025810999508924);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.013582828506600017));
                c.brake.set(Controller.proportionalFeedforward(0.011545404230610014));

                c.headingFeedback.set(Controller.proportional(23.16036530573261));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.10837679923826098, 0.031516260053852545));

                c.linearBrakeCoefficients.set(Matrix.diag(0.040438668898360425, 0.06896713233777459));
                c.quadraticBrakeCoefficients.set(Matrix.diag(6.177386490455197E-4, 0.001127295592119859));

                c.maxAchievableForwardVelocity.set(73.52962628544806);
                c.maxAchievableStrafeVelocity.set(60.68159855921948);
                c.naturalForwardDeceleration.set(52.49753740649758);
                c.naturalStrafeDeceleration.set(79.51279181439291);
            }
    );

    @Nullable
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
        // return new Follower(
                // new PinpointLocalizer(h, localizerConfig),
                // new Mecanum(h, drivetrainConfig),
                // new Foresight(foresightConfig)
        // );
    }
}