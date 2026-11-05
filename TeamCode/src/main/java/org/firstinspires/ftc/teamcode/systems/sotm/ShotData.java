package org.firstinspires.ftc.teamcode.systems.sotm;

public class ShotData {
    private ShotData() {}

    /** distance (in), flywheel ticks/s, hood angle (deg - removed), time of flight (s). */
    public static void load(ShotTable table) {
        table.add(92.3, 1270, 35, 1.2);
        // table.add(83, 1220)
        table.add(71.4, 1240, 34.5, 1);
        table.add(43.7, 1150, 23, 0.3);
        table.add(60, 1195, 37, 0.8);
        table.add(55, 1170, 27, 0.65);
    }
}
