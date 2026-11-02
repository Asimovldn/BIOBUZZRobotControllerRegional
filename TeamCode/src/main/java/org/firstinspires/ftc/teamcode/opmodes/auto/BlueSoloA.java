package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Context;

@Autonomous(name = "Blue Solo A", group = "Auto")
public class BlueSoloA extends RedSoloA {

    @Override
    public Context.Alliance allianceForOpMode() {
        return Context.Alliance.BLUE;
    }
}
