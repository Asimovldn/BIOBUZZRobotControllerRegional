package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
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
                instant(() -> ramp.set(ramp.off())),
                waitMs(20),
                spin(true),
                shoot(),
                stopSpinUnlessKept(),

                flowerRoutine("Audience"),

                instant(() -> {
                    runtime.switchLocation();
                    autoSpin = true;
                }),
                setIntake(1),
                instant(() -> ramp.set(ramp.on())),
                followPath("toScoring", AutoTune.longPathTimeoutMs, AutoTune.handoffToleranceIn),
                instant(() -> ramp.alter()),
                shoot(),
                stopSpinUnlessKept(),

                instant(() -> ramp.set(ramp.on())),
                waitMs(40),

                flowerRoutine("Scoring"),

                setIntake(1),
                waitMs(300),
                spin(true),
                shoot(AutoTune.shootMs + AutoTune.finalExtraFeedMs),
                stopSpinUnlessKept(),

                parkRoutine(),
                instant(() -> ramp.set(ramp.off())),
                stopSpin(),

                waitUntil(() -> shooter.atSpeed(0, 40)),
                setRampMax()
        );
    }
}