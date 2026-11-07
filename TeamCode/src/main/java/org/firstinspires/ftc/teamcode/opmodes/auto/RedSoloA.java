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
                instant(() -> hood.set(hood.off())),

                waitMs(400),

                shoot(),
                stopSpinUnlessKept(),

                // garden routine
                flowerRoutine("Audience"),

                instant(() -> {
                    runtime.switchLocation();
                    autoSpin = true;
                }),

                // second cycle

                setIntake(1),
                instant(() -> hood.set(hood.on())),
                followPath("fromFlowerToFlower"),
                waitMs(AutoTune.flowerRampMs),
                shoot(),
                stopSpinUnlessKept(),

                // second flower

                flowerRoutine("Scoring"),

                // third cycle
                shoot(),
                setIntake(1),
                waitMs(150),
                shoot(700),
                stopSpinUnlessKept(),

                // park

                parkRoutine(),
                instant(() -> hood.maxUp()),
                stopSpin()
        );
    }
}
