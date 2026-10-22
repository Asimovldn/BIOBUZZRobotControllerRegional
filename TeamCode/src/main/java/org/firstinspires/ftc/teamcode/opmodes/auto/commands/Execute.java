package org.firstinspires.ftc.teamcode.opmodes.auto.commands;

import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.RobotOpMode;

public class Execute {
    private final Follower follower;

    public Execute(Follower flwr) {
        follower = flwr;
    }

    public CommandBuilder run(Path path) {
        return follow(follower, path);
    }
}
