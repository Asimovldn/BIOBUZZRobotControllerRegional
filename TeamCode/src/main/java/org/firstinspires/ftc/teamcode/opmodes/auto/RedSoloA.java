package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.pedropathing.ivy.commands.Commands.instant;
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
                instant(() -> ramp.set(ramp.off())),
                shoot(),
                stopSpinUnlessKept(),

                flowerRoutine("Audience"),

                instant(() -> {
                    runtime.switchLocation();
                    autoSpin = true;
                }),
                setIntake(1),
                instant(() -> ramp.set(ramp.on())),
                followPath("fromFlowerToFlower", AutoTune.longPathTimeoutMs, AutoTune.handoffToleranceIn),
                shoot(),
                stopSpinUnlessKept(),

                flowerRoutine("Scoring"),

                spin(true),
                shoot(AutoTune.shootMs + AutoTune.finalExtraFeedMs),
                stopSpinUnlessKept(),

                parkRoutine(),
                instant(() -> ramp.maxUp()),
                stopSpin()
        );
    }
}