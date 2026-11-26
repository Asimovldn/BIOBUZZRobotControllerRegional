package org.firstinspires.ftc.teamcode.opmodes.tests;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Context;

@TeleOp
public class BlueCalibrate extends RedCalibrate {
    @Override
    public Context.Alliance allianceForOpMode() {
        return Context.Alliance.BLUE;
    }
}
