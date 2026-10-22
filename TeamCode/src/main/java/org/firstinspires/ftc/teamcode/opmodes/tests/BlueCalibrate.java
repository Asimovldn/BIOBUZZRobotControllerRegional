package org.firstinspires.ftc.teamcode.opmodes.tests;


import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Context;
import org.firstinspires.ftc.teamcode.RobotOpMode;
import org.firstinspires.ftc.teamcode.systems.hardware.Turret;

@TeleOp
public class BlueCalibrate extends RedCalibrate {
    @Override
    public Context.Alliance allianceForOpMode() {
        return Context.Alliance.BLUE;
    }
}
