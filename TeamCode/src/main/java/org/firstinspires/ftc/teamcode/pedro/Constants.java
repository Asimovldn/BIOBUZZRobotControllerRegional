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
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-6.5);
        c.yPodOffset.set(0.0);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.3051747091306715);
                Controller secondaryTranslationalForward = Controller.proportional(0.11275390060537309);
                Controller primaryTranslationalLateral = Controller.proportional(0.4190608620388987);
                Controller secondaryTranslationalLateral = Controller.proportional(0.15483179101091188);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.02029900048517451));
                c.brake.set(Controller.proportionalFeedforward(0.017254150412398334));

                c.headingFeedback.set(Controller.proportional(4.092119959302333));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.03798998389221657, 0.008402778275129039));

                c.linearBrakeCoefficients.set(Matrix.diag(0.09676533977555674, 0.06145942514436504));
                c.quadraticBrakeCoefficients.set(Matrix.diag(-5.231886196004381E-5, 0.001308033103816754));

                c.maxAchievableForwardVelocity.set(52.539850570564646);
                c.maxAchievableStrafeVelocity.set(45.559417552555246);
                c.naturalForwardDeceleration.set(54.29967253734653);
                c.naturalStrafeDeceleration.set(94.22507253448138);
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