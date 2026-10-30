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
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.systems.sotm.SOTM;
import org.firstinspires.ftc.teamcode.systems.sotm.ShotData;
import org.firstinspires.ftc.teamcode.systems.sotm.ShotSolver;

import java.util.function.BooleanSupplier;

public class RedSoloA extends Auto {

    SOTM sotm;
    Context.HiveLocations locs;
    @Override
    public void setup() {
        sotm = new SOTM();
        ShotData.load(sotm.table);
        locs = new Context.HiveLocations(runtime);
    }

    public String code() {
        return "A";
    }

    @Override
    public void periodic() {
        if (sotm == null) return;

        Pose pose = follower.pose();
        sotm.update(pose, follower.velocity());

        Pose goal = locs.get();
        if (goal == null) return;

        ShotSolver.Solution shot = sotm.solve(pose, goal);
        if (!shot.valid) return;

        getTurret().setTrackAngle(sotm.turretAngleDeg(shot));
        hood.setServoToPoses(hood.angleToServo(shot.hood));
    }

    @Nullable
    @Override
    public Command routine() {
        return sequential(
                instant(() -> getTurret().setMode(Turret.Mode.TRACK)),
                instant(() -> runtime.location = "Audience"),
                instant(() -> shooter.setTargetVelocity(-sotm.solve(follower.pose(), locs.get()).velocity)),
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
