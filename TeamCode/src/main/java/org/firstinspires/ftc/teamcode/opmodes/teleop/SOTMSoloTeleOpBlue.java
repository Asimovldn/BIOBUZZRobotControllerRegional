package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Context;

@TeleOp(name = "SOTM Solo Blue", group = "TeleOp")
public class SOTMSoloTeleOpBlue extends SOTMSoloTeleOp {
    @Override
    public Context.Alliance allianceForOpMode() {
        return Context.Alliance.BLUE;
    }
}
