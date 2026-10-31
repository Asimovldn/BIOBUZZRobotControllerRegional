package org.firstinspires.ftc.teamcode.pedro;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.Path;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.List;

public class Drawing {
    public static final double ROBOT_RADIUS = 9;

    // Pedro field: 144x144 in, origin at bottom-left.
    // FTC Dashboard field overlay: origin at field center.
    // (Equivalent of Panels' PEDRO_PATHING preset offsets.)
    private static final double FIELD_OFFSET_X = 72;
    private static final double FIELD_OFFSET_Y = 72;

    private static final int MAX_HISTORY = 500;
    private static final int PATH_SAMPLES = 100;

    private static final String ROBOT_LOOK = "#3F51B5";
    private static final String HISTORY_LOOK = "#4CAF50";
    private static final String POSE_LOOK = "#6693F5";

    private static final Deque<Pose> poseHistory = new ArrayDeque<>();

    private static TelemetryPacket packet = new TelemetryPacket();

    public static void init() {
        packet = new TelemetryPacket();
        clearPoseHistory();
    }

    public static void drawDebug(Follower follower) {
        if (follower == null) {
            return;
        }

        Pose currentPose = follower.pose();

        addPoseToHistory(currentPose);

        Path currentPath = follower.currentPath();

        if (currentPath != null) {
            drawPath(currentPath, ROBOT_LOOK);
            drawRobot(follower.closestPose(), ROBOT_LOOK);
        }

        drawPoseHistory(new ArrayList<>(poseHistory), HISTORY_LOOK);

        drawRobot(currentPose, HISTORY_LOOK);

        sendPacket();
    }

    public static void drawRobot(Pose pose, String color) {
        if (!isValidPose(pose)) {
            return;
        }

        Canvas canvas = packet.fieldOverlay();

        double x = pose.x();
        double y = pose.y();
        double heading = pose.heading();

        canvas.setStroke(color);
        canvas.setStrokeWidth(1);
        canvas.strokeCircle(fx(x), fy(y), ROBOT_RADIUS);

        double headingX = Math.cos(heading) * ROBOT_RADIUS;
        double headingY = Math.sin(heading) * ROBOT_RADIUS;

        double x1 = x + headingX / 2;
        double y1 = y + headingY / 2;
        double x2 = x + headingX;
        double y2 = y + headingY;

        canvas.strokeLine(fx(x1), fy(y1), fx(x2), fy(y2));
    }

    public static void drawRobot(Pose pose) {
        drawRobot(pose, ROBOT_LOOK);
    }

    public static void drawAim(Path path) {
        drawPath(path, POSE_LOOK);
    }

    public static void drawPath(Path path, String color) {
        if (path == null || path.curve == null) {
            return;
        }

        Canvas canvas = packet.fieldOverlay();
        canvas.setStroke(color);
        canvas.setStrokeWidth(1);

        List<Double> xs = new ArrayList<>();
        List<Double> ys = new ArrayList<>();

        for (int i = 0; i <= PATH_SAMPLES; i++) {
            double t = i / (double) PATH_SAMPLES;

            Vector2D point = path.curve.get(t);

            if (point == null
                    || Double.isNaN(point.x())
                    || Double.isNaN(point.y())
                    || Double.isInfinite(point.x())
                    || Double.isInfinite(point.y())) {
                // Break the line at invalid points, like the Panels version.
                flushPolyline(canvas, xs, ys);
                continue;
            }

            xs.add(fx(point.x()));
            ys.add(fy(point.y()));
        }

        flushPolyline(canvas, xs, ys);
    }

    public static void drawPoseHistory(List<Pose> poses, String color) {
        if (poses == null || poses.isEmpty()) {
            return;
        }

        Canvas canvas = packet.fieldOverlay();
        canvas.setStroke(color);
        canvas.setStrokeWidth(1);

        List<Double> xs = new ArrayList<>();
        List<Double> ys = new ArrayList<>();

        for (Pose pose : poses) {
            if (!isValidPose(pose)) {
                flushPolyline(canvas, xs, ys);
                continue;
            }

            xs.add(fx(pose.x()));
            ys.add(fy(pose.y()));
        }

        flushPolyline(canvas, xs, ys);
    }

    public static void drawPoseHistory(List<Pose> poses) {
        drawPoseHistory(poses, HISTORY_LOOK);
    }

    private static void flushPolyline(Canvas canvas, List<Double> xs, List<Double> ys) {
        if (xs.size() >= 2) {
            double[] xArr = new double[xs.size()];
            double[] yArr = new double[ys.size()];
            for (int i = 0; i < xArr.length; i++) {
                xArr[i] = xs.get(i);
                yArr[i] = ys.get(i);
            }
            canvas.strokePolyline(xArr, yArr);
        }
        xs.clear();
        ys.clear();
    }

    private static double fx(double pedroX) {
        return pedroX - FIELD_OFFSET_X;
    }

    private static double fy(double pedroY) {
        return pedroY - FIELD_OFFSET_Y;
    }

    private static void addPoseToHistory(Pose pose) {
        if (!isValidPose(pose)) {
            return;
        }

        if (poseHistory.size() >= MAX_HISTORY) {
            poseHistory.removeFirst();
        }

        poseHistory.addLast(pose);
    }

    public static void clearPoseHistory() {
        poseHistory.clear();
    }

    private static boolean isValidPose(Pose pose) {
        if (pose == null) {
            return false;
        }

        double x = pose.x();
        double y = pose.y();
        double heading = pose.heading();

        return !Double.isNaN(x)
                && !Double.isNaN(y)
                && !Double.isNaN(heading)
                && !Double.isInfinite(x)
                && !Double.isInfinite(y)
                && !Double.isInfinite(heading);
    }

    public static void sendPacket() {
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
        packet = new TelemetryPacket();
    }

    public static void drawPose(Pose pose) {
        if (!isValidPose(pose)) {
            return;
        }

        Canvas canvas = packet.fieldOverlay();

        double x = pose.x();
        double y = pose.y();
        double heading = pose.heading();

        canvas.setStroke(POSE_LOOK);
        canvas.setStrokeWidth(1);
        canvas.strokeCircle(fx(x), fy(y), ROBOT_RADIUS);

        double headingX = Math.cos(heading) * ROBOT_RADIUS;
        double headingY = Math.sin(heading) * ROBOT_RADIUS;

        double x1 = x + headingX / 2;
        double y1 = y + headingY / 2;
        double x2 = x + headingX;
        double y2 = y + headingY;

        canvas.strokeLine(fx(x1), fy(y1), fx(x2), fy(y2));
    }
}