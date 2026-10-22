package org.firstinspires.ftc.teamcode.systems.hardware;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Context;
import org.firstinspires.ftc.teamcode.systems.hardware.core.Mechanism;

public class Intake extends Mechanism {
    private final Context.IntakeConstants constants = new Context.IntakeConstants();

    @Override
    public void setup() {
        motor = runtime.hardwareMap.get(DcMotorEx.class, "intake");
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(constants.kP, 0, 0, constants.kF));
    }
}
