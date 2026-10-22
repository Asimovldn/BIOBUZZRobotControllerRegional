package org.firstinspires.ftc.teamcode.systems.sotm;

import static org.firstinspires.ftc.teamcode.util.Util.lerp;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class ShotTable {
    public static class Entry {
        public final double velocity, hood, tof;

        public Entry(double velocity, double hood, double tof) {
            this.velocity = velocity;
            this.hood = hood;
            this.tof = tof;
        }
    }

    private static class Row {
        final double distance;
        final Entry entry;

        Row(double distance, Entry entry) {
            this.distance = distance;
            this.entry = entry;
        }
    }

    private final List<Row> rows = new ArrayList<>();
    private Entry fallback = new Entry(0, 0, 0.45);

    /** distance in inches, velocity in flywheel ticks/s, tof in seconds. */
    public ShotTable add(double distance, double velocity, double hood, double tof) {
        rows.add(new Row(distance, new Entry(velocity, hood, tof)));
        Collections.sort(rows, (a, b) -> Double.compare(a.distance, b.distance));
        return this;
    }

    public ShotTable fallback(double velocity, double hood, double tof) {
        fallback = new Entry(velocity, hood, tof);
        return this;
    }

    public boolean isEmpty() {
        return rows.isEmpty();
    }

    public int size() {
        return rows.size();
    }

    public Entry get(double distance) {
        int n = rows.size();
        if (n == 0) return fallback;
        if (distance <= rows.get(0).distance) return rows.get(0).entry;
        if (distance >= rows.get(n - 1).distance) return rows.get(n - 1).entry;

        for (int i = 1; i < n; i++) {
            Row b = rows.get(i);
            if (distance > b.distance) continue;

            Row a = rows.get(i - 1);
            double span = b.distance - a.distance;
            double t = span < 1e-9 ? 0 : (distance - a.distance) / span;
            return new Entry(
                    lerp(a.entry.velocity, b.entry.velocity, t),
                    lerp(a.entry.hood, b.entry.hood, t),
                    lerp(a.entry.tof, b.entry.tof, t));
        }

        return rows.get(n - 1).entry;
    }
}
