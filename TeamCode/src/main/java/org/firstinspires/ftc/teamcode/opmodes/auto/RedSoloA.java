package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.jetbrains.annotations.Nullable;

@Autonomous(name = "Red Solo A", group = "Auto")
public class RedSoloA extends Auto {

    @Override
    public String code() {
        return "A";
    }

    @Nullable
    @Override
    public Command routine() {
        return sequential(
                spin(true),
                shoot(),
                stopSpinUnlessKept(),
                setIntake(1),
                followPath("toGarden"),
                waitMs(AutoTune.gardenDwellMs),
                instant(() -> {
                    runtime.switchLocation();
                    autoSpin = true;
                }),
                followPath("toScoring"),
                shoot(),
                spin(false)
        );
    }
}
