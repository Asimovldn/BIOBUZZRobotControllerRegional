package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Context;

@TeleOp(name = "SOTM Blue", group = "TeleOp")
public class SOTMTeleOpBlue extends SOTMTeleOp {
    @Override
    public Context.Alliance allianceForOpMode() {
        return Context.Alliance.BLUE;
    }
}
