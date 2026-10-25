package org.firstinspires.ftc.teamcode.subsystems.shooter;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.subsystems.vision.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.vision.CellCluster;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.nio.charset.StandardCharsets;
import java.util.Locale;

/** Persistent append-only CSV storage for driver-recorded shot calibration points. */
public final class ShotCalibrationStore {
    public static final String FILE_NAME = "shot-calibration.csv";
    private static final String ONE_TIME_CLEAR_MARKER =
            "shot-calibration-cleared-20261025-v1.marker";
    private static final String HEADER =
            "timestamp_ms,alliance,cell,distance_cm,target_speed_ticks_per_s,pitch_position";
    private final File file;

    public ShotCalibrationStore() {
        file = AppUtil.getInstance().getSettingsFile(FILE_NAME);
    }

    public synchronized void append(
            Alliance alliance,
            CellCluster cell,
            double distanceCm,
            double targetSpeed,
            double pitchPosition
    ) throws IOException {
        if (alliance == null || cell == null
                || !Double.isFinite(distanceCm) || distanceCm < 0.0
                || !Double.isFinite(targetSpeed) || targetSpeed < 0.0
                || !Double.isFinite(pitchPosition)
                || pitchPosition < 0.0 || pitchPosition > 1.0) {
            throw new IllegalArgumentException("Invalid shot calibration point");
        }

        File parent = file.getParentFile();
        if (parent != null && !parent.exists() && !parent.mkdirs()) {
            throw new IOException("Cannot create calibration directory");
        }
        boolean writeHeader = !file.exists() || file.length() == 0;
        try (BufferedWriter writer = new BufferedWriter(new OutputStreamWriter(
                new FileOutputStream(file, true), StandardCharsets.UTF_8))) {
            if (writeHeader) {
                writer.write(HEADER);
                writer.newLine();
            }
            writer.write(String.format(Locale.US, "%d,%s,%s,%.3f,%.1f,%.4f",
                    System.currentTimeMillis(), alliance, cell,
                    distanceCm, targetSpeed, pitchPosition));
            writer.newLine();
        }
    }

    public File getFile() {
        return file;
    }

    /** Adds a required calibration point only when that exact distance is not already stored. */
    public synchronized boolean ensurePoint(
            Alliance alliance,
            CellCluster cell,
            double distanceCm,
            double targetSpeed,
            double pitchPosition
    ) throws IOException {
        for (ShotLookupTable.Setpoint point : ShotLookupTable.load(file).getPoints()) {
            if (Math.abs(point.distanceCm - distanceCm) < 0.001) return false;
        }
        append(alliance, cell, distanceCm, targetSpeed, pitchPosition);
        return true;
    }

    /** Clears the pre-existing table once, without deleting points recorded after this update. */
    public synchronized boolean clearExistingDataOnce() throws IOException {
        File marker = AppUtil.getInstance().getSettingsFile(ONE_TIME_CLEAR_MARKER);
        if (marker.exists()) return false;

        if (file.exists() && !file.delete()) {
            throw new IOException("Cannot delete old calibration CSV");
        }

        File parent = marker.getParentFile();
        if (parent != null && !parent.exists() && !parent.mkdirs()) {
            throw new IOException("Cannot create calibration reset marker directory");
        }
        try (FileOutputStream ignored = new FileOutputStream(marker)) {
            // The persistent marker ensures later OpMode starts keep newly recorded points.
        }
        return true;
    }
}
