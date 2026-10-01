package org.firstinspires.ftc.teamcode.opmodes.auto.commands;

import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.systems.Bot;

public class Execute {
    private final Follower follower;
    private final Bot robot;

    public Execute(Follower flwr, Bot bot) {
        follower = flwr;
        robot = bot;
    }

    public CommandBuilder run(Path path) {
        return follow(follower, path);
    }
}
