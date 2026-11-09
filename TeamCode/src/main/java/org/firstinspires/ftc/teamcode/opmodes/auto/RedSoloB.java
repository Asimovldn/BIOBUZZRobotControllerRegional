package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.jetbrains.annotations.Nullable;

@Autonomous(name = "Red Solo B", group = "Auto")
public class RedSoloB extends Auto {

    @Override
    public String code() {
        return "B";
    }

    @Nullable
    @Override
    public Command routine() {
        return sequential(
                spin(true),
                instant(() -> ramp.set(ramp.on())),
                shoot(),
                stopSpinUnlessKept(),

                gardenRoutine(),

                instant(() -> {
                    autoSpin = true;
                }),
                setIntake(1),
                instant(() -> ramp.set(ramp.on())),
                followPath("fromGardenScore", AutoTune.longPathTimeoutMs, AutoTune.handoffToleranceIn),
                shoot(),
                stopSpinUnlessKept(),

                gated(AutoTune.flowerMinSec, sequential(
                        flowerRoutine("Scoring"),
                        spin(true),
                        shoot(AutoTune.shootMs + AutoTune.finalExtraFeedMs),
                        stopSpinUnlessKept()
                ), () -> flowerStageDone = true),

                onlyIf(() -> flowerStageDone, parkRoutine()),
                onlyIf(() -> !flowerStageDone, parkFromScoringRoutine()),

                finishRoutine()
        );
    }
}
