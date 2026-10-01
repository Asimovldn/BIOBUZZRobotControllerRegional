package org.firstinspires.ftc.teamcode.opmodes.auto.commands;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.groups.Groups.parallel;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.storage.Storage;

public class PathBuilder {
    private final PoseFactory poseFactory = PoseFactory.degrees();
    public PoseFactory getFactory() {
        return poseFactory;
    }

    public void mirrorPose(Storage.Alliance alliance) {
        if (alliance == Storage.Alliance.RED) {
            poseFactory.mirrorX(72.0);
            poseFactory.mirrorY(72.0);
        }
    }

    // Make Poses Below

    public final Pose startPoseA = poseFactory.of(83, 8.65, 90);
    public final Pose parkPoseA = poseFactory.of(133, 30, 180);

    // Insert Paths Below

    public Path startToLeave() {
        return line(startPoseA, parkPoseA).linear(startPoseA, parkPoseA);
    }
}
