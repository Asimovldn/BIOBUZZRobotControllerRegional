package org.firstinspires.ftc.teamcode.systems.sotm;

public class ShotData {
    private ShotData() {}

    /** distance (in), flywheel ticks/s, time of flight (s). The hood is fixed, see Shooter.HOOD_DEG. */
    public static void load(ShotTable table) {
        table.add(92.3, 1270, 1.2);
        table.add(82, 1240, 1.2);
        table.add(71.4, 1170, 1);
        table.add(69, 1150, 1);
        table.add(60, 1140, 0.8);
        table.add(55, 1125, 0.65);
        table.add(43.7, 1095, 0.3);
    }
}
