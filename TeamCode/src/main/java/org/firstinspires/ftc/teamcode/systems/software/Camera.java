package org.firstinspires.ftc.teamcode.systems.software;

import static java.lang.Math.toDegrees;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.util.LazyMath;
import org.jetbrains.annotations.Nullable;

public class Camera {
    @Nullable
    private final Limelight3A cam;

    private double lastTagAngle, lastRobotHeading, lastPredict;

    private boolean hasSeenTag = false;

    public Camera(HardwareMap hm) {
        cam = hm.tryGet(Limelight3A.class, "limelight");
    }

    @Nullable
    public Camera start() {
        if (cam == null) return null;

        cam.start();
        cam.pipelineSwitch(0);

        return this;
    }

    public void stop() {
        if (cam == null) return;
        cam.shutdown();
    }

    public double getX(int APRIL_TAG_ID) {
        if (cam == null) return -1;

        LLResult res = cam.getLatestResult();

        if (!res.isValid()) return -1;

        for (LLResultTypes.FiducialResult result : res.getFiducialResults()) {
            if (result.getFiducialId() != APRIL_TAG_ID) continue;

            return result.getTargetXDegrees();
        }

        return -1;
    }

    /**
     * Gets the robot pose in Pedro Pathing coordinates based on all tags.
     */
    public Pose get() {
        if (cam == null) return new Pose(0, 0, 0);

        LLResult res = cam.getLatestResult();

        if (!res.isValid()) return new Pose(0, 0, 0);

        Pose3D pos = res.getBotpose();

        return LazyMath.fromFTCPose(pos);
    }

    /**
     * Gets the robot pose in Pedro Pathing coordinates.
     * @param APRIL_TAG_ID the Tag to be looking for.
     */
    public Pose get(double APRIL_TAG_ID) {
        if (cam == null) return new Pose(0, 0, 0);

        LLResult res = cam.getLatestResult();

        if (!res.isValid()) return new Pose(0, 0, 0);

        for (LLResultTypes.FiducialResult fres : res.getFiducialResults()) {
            if (fres.getFiducialId() != APRIL_TAG_ID) continue;

            Pose3D pos = fres.getRobotPoseFieldSpace();

            return LazyMath.fromFTCPose(pos);
        }

        return new Pose(0, 0, 0);
    }

    public void reset() {
        if (cam == null) return;
        cam.reloadPipeline();
    }

    public double getY(int APRIL_TAG_ID) {
        if (cam == null) return -1;

        LLResult res = cam.getLatestResult();

        for (LLResultTypes.FiducialResult result : res.getFiducialResults()) {
            if (result.getFiducialId() != APRIL_TAG_ID) continue;

            Pose3D pose = result.getTargetPoseCameraSpace();

            double x = pose.getPosition().x;
            double y = pose.getPosition().y;
            double z = pose.getPosition().z;

            return Math.sqrt(x * x + y * y + z * z);
        }

        return 0;
    }

    @Nullable
    public Pose3D getPose(int APRIL_TAG_ID, double heading) {
        if (cam == null) return null;

        cam.updateRobotOrientation(toDegrees(heading));


        LLResult res = cam.getLatestResult();

        if (!res.isValid()) return null;

        return res.getBotpose();
    }

    public double getY(int APRIL_TAG_ID, double heading) {
        if (cam == null) return 0;
        cam.updateRobotOrientation(toDegrees(heading));

        boolean found = false;

        LLResult llResult = cam.getLatestResult();
        if (llResult != null && llResult.isValid()) {

            for (LLResultTypes.FiducialResult res : llResult.getFiducialResults()) {
                if (res.getFiducialId() == APRIL_TAG_ID) {
                    found = true;
                }
            }
            if (!found) return 0;

            return getDistanceFromTag(llResult.getTa());
        }

        return 0;
    }

    public double getDistanceFromTag(double ta) {
        double scale = 21427.2553352915;
        return Math.sqrt(scale) / Math.sqrt(ta);
    }
}
