package org.firstinspires.ftc.teamcode.systems.sotm;

public final class ShotData {
    private ShotData() {}

    /** distance (in), flywheel ticks/s, hood angle (deg), time of flight (s). Shared by TeleOp and Auto. */
    public static void load(ShotTable table) {
        table.add(82, 1270, 34.5, 1);
        table.add(43, 1190, 23, 0.3);
        table.add(64.5, 1230, 37, 0.8);
        table.add(55, 1240, 27, 0.65);
    }
}
