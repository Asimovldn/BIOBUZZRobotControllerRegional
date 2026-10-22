package org.firstinspires.ftc.teamcode.systems.hardware;


import static com.pedropathing.ivy.groups.Groups.parallel;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Context;
import org.firstinspires.ftc.teamcode.systems.hardware.core.Mechanism;
import org.firstinspires.ftc.teamcode.util.PIDFController;


public class Shooter extends Mechanism {
    private DcMotorEx backmotor, foremotor;
    private double targetSpeed;
    private final Context.ShooterConstants.BackShooter backConstants = new Context.ShooterConstants.BackShooter();
    private Context.ShooterConstants.Coefficients coefficients;
    private PIDFController control;

    public Shooter() {
        super(360 * 28);
    }

    @Override
    public void setup() {
        HardwareMap hw = runtime.hardwareMap;

        backmotor = hw.tryGet(DcMotorEx.class, "foreShooter");
        foremotor = hw.tryGet(DcMotorEx.class, "backShooter");

        backmotor.setDirection(DcMotorSimple.Direction.FORWARD);

        coefficients = new Context.ShooterConstants.Coefficients();

        control = new PIDFController(coefficients.kP, coefficients.kI, coefficients.kD, coefficients.kV);

        /*
        backmotor.getVelocityConstants().setKS(backConstants.kS);
        backmotor.getVelocityConstants().setKV(backConstants.kV);

        foremotor.getVelocityConstants().setKS(foreConstants.kS);
        foremotor.getVelocityConstants().setKV(foreConstants.kV);
         */
    }

    /**
     * In Order:
     * [backMotor Speed, foreMotor Speed]
     * Speed()[0], Speed()[1]
     * @return
     */
    @Override
    public double Speed() {
        double[] vels = new double[2];
        vels[0] = backmotor.getVelocity();
        vels[1] = foremotor.getVelocity();

        return (vels[0]);
    }

    public double calculatePIDF(double err) {
        return control.calculate(err);
    }

    public void prepareLaunch(double strength) {
        foremotor.setVelocity(control.calculate((targetSpeed * strength) - foremotor.getVelocity()));
    }

    public Command enableLaunch() {
        return Command.build()
                .setExecute(() -> {
                    backmotor.setPower(-0.8);
                })
                .requiring(this);
    }

    public Command fullBrake() {
        return parallel(
                disableLaunch()
        );
    }

    @Override
    public void periodic() {}

    public Command disableLaunch() {
        return Command.build()
                .setExecute(() ->
                    backmotor.setPower(0)
                )
                .setDone(() -> backmotor.getPower() < 0.1);
    }
}
