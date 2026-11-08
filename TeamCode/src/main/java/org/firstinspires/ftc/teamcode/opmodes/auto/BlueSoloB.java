package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Context;

@Autonomous(name = "Blue Solo B", group = "Auto")
public class BlueSoloB extends RedSoloB {

    @Override
    public Context.Alliance allianceForOpMode() {
        return Context.Alliance.BLUE;
    }
}
