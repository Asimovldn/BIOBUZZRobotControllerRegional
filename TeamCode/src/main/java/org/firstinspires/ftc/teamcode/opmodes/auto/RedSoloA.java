package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import androidx.annotation.Nullable;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.Context;
import org.firstinspires.ftc.teamcode.systems.hardware.Turret;
import org.firstinspires.ftc.teamcode.systems.sotm.SOTM;

import java.util.function.BooleanSupplier;

public class RedSoloA extends Auto {

    SOTM sotm;
    Context.HiveLocations locs;
    @Override
    public void setup() {
        sotm = new SOTM();
        locs = new Context.HiveLocations(runtime);
    }

    @Override
    public void periodic() {
        sotm.update(follower.pose());
    }

    public void fill() {
        sotm.table.add(85, 1300, 0, 1);
        sotm.table.add(33, 1100, 0, 0.3);
        sotm.table.add(73, 1250, 0, 0.8);
        sotm.table.add(54, 1150, 0, 0.65);
    }

    @Nullable
    @Override
    public Command routine() {
        return sequential(
                instant(() -> getTurret().setMode(Turret.Mode.TRACK)),
                instant(() -> runtime.location = "Audience"),
                instant(() -> shooter.setTargetVelocity(sotm.solve(follower.pose(), locs.get()).velocity)),
                waitUntil(shooter::isReady),
                shooter.enableLaunch(),
                instant(() -> intake.setThrottle(1)),
                waitMs(500),
                instant(() -> getTurret().setMode(Turret.Mode.OFF)),
                shooter.disableLaunch(),
                shooter.fullBrake(),
                follow(getFollower(), Operator.get("toGarden")),
                instant(() -> getTurret().setMode(Turret.Mode.TRACK)),
                instant(() -> runtime.switchLocation()),
                follow(getFollower(), Operator.get("toScoringRed"))
        );
    }
}
