package org.firstinspires.ftc.teamcode.subsystems.shooter;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.TreeMap;

/** Loads recorded CSV points and linearly interpolates speed and pitch by distance. */
public final class ShotLookupTable {
    private final List<Setpoint> points;

    private ShotLookupTable(List<Setpoint> points) {
        this.points = points;
    }

    public static ShotLookupTable fromPoints(Collection<Setpoint> source) {
        Map<Double, Setpoint> latestPointAtDistance = new TreeMap<>();
        if (source != null) {
            for (Setpoint point : source) {
                if (point != null
                        && Double.isFinite(point.distanceCm) && point.distanceCm >= 0.0
                        && Double.isFinite(point.targetSpeed) && point.targetSpeed >= 0.0
                        && Double.isFinite(point.pitchPosition)) {
                    latestPointAtDistance.put(point.distanceCm, point);
                }
            }
        }
        return new ShotLookupTable(new ArrayList<>(latestPointAtDistance.values()));
    }

    public static ShotLookupTable load(File file) throws IOException {
        Map<Double, Setpoint> latestPointAtDistance = new TreeMap<>();
        if (file != null && file.exists()) {
            try (BufferedReader reader = new BufferedReader(new InputStreamReader(
                    new FileInputStream(file), StandardCharsets.UTF_8))) {
                String line;
                while ((line = reader.readLine()) != null) {
                    String[] fields = line.split(",", -1);
                    if (fields.length < 6 || "timestamp_ms".equals(fields[0])) continue;
                    try {
                        double distanceCm = Double.parseDouble(fields[3]);
                        double speed = Double.parseDouble(fields[4]);
                        double pitch = Double.parseDouble(fields[5]);
                        if (Double.isFinite(distanceCm) && distanceCm >= 0.0
                                && Double.isFinite(speed) && speed >= 0.0
                                && Double.isFinite(pitch) && pitch >= 0.0 && pitch <= 1.0) {
                            // The last appended row wins when an exact distance is re-calibrated.
                            latestPointAtDistance.put(
                                    distanceCm, new Setpoint(distanceCm, speed, pitch));
                        }
                    } catch (NumberFormatException ignored) {
                        // Preserve usable rows if a manually edited CSV row is malformed.
                    }
                }
            }
        }
        return fromPoints(latestPointAtDistance.values());
    }

    public Optional<Setpoint> lookup(double distanceCm) {
        if (!Double.isFinite(distanceCm) || points.isEmpty()) return Optional.empty();
        if (points.size() == 1) return Optional.of(points.get(0));

        if (distanceCm <= points.get(0).distanceCm) {
            return Optional.of(interpolateOrExtrapolate(
                    points.get(0), points.get(1), distanceCm));
        }
        Setpoint last = points.get(points.size() - 1);
        if (distanceCm >= last.distanceCm) {
            return Optional.of(interpolateOrExtrapolate(
                    points.get(points.size() - 2), last, distanceCm));
        }

        for (int index = 1; index < points.size(); index++) {
            Setpoint upper = points.get(index);
            if (distanceCm <= upper.distanceCm) {
                Setpoint lower = points.get(index - 1);
                return Optional.of(interpolateOrExtrapolate(lower, upper, distanceCm));
            }
        }
        return Optional.of(last);
    }

    public int size() {
        return points.size();
    }

    public List<Setpoint> getPoints() {
        return Collections.unmodifiableList(new ArrayList<>(points));
    }

    private static double lerp(double from, double to, double ratio) {
        return from + (to - from) * ratio;
    }

    private static Setpoint interpolateOrExtrapolate(
            Setpoint lower, Setpoint upper, double distanceCm) {
        double span = upper.distanceCm - lower.distanceCm;
        if (!Double.isFinite(span) || Math.abs(span) < 1e-9) return lower;
        double ratio = (distanceCm - lower.distanceCm) / span;
        return new Setpoint(
                distanceCm,
                lerp(lower.targetSpeed, upper.targetSpeed, ratio),
                lerp(lower.pitchPosition, upper.pitchPosition, ratio));
    }

    public static final class Setpoint {
        public final double distanceCm;
        public final double targetSpeed;
        public final double pitchPosition;

        public Setpoint(double distanceCm, double targetSpeed, double pitchPosition) {
            this.distanceCm = distanceCm;
            this.targetSpeed = targetSpeed;
            this.pitchPosition = pitchPosition;
        }
    }
}
