package org.firstinspires.ftc.teamcode.tasks;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShotLookupTable;
import org.firstinspires.ftc.teamcode.subsystems.vision.CellCluster;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * Produces shooter speed and pitch from the current field-odometry distance.
 *
 * <p>This hook intentionally has no dependency on a current Limelight frame. Vision corrects the
 * Pinpoint field pose whenever it is valid; between vision frames (or while vision is unavailable),
 * Pinpoint continues updating from that last corrected pose.</p>
 */
public final class DistanceShotTaskHook {
    private static final List<ShotLookupTable.Setpoint> DEFAULT_CALIBRATION = Arrays.asList(
            new ShotLookupTable.Setpoint(70.0, 1600.0, 0.4000),
            new ShotLookupTable.Setpoint(113.144, 1650.0, 0.1600)
    );

    private final Drive drive;
    private final Supplier<CellCluster> targetSupplier;
    private final File calibrationFile;
    private ShotLookupTable calibration;
    private ShotLookupTable.Setpoint setpoint;
    private double distanceCm = Double.NaN;
    private boolean updatedFromOdometry;

    public DistanceShotTaskHook(
            Drive drive, Supplier<CellCluster> targetSupplier, File calibrationFile) {
        this.drive = drive;
        this.targetSupplier = targetSupplier;
        this.calibrationFile = calibrationFile;
        reloadCalibration();
    }

    /** Merge persistent driver-recorded points over the two built-in competition points. */
    public void reloadCalibration() {
        List<ShotLookupTable.Setpoint> merged = new ArrayList<>(DEFAULT_CALIBRATION);
        if (calibrationFile != null) {
            try {
                merged.addAll(ShotLookupTable.load(calibrationFile).getPoints());
            } catch (IOException ignored) {
                // The two built-in points remain usable if persistent storage is unavailable.
            }
        }
        calibration = ShotLookupTable.fromPoints(merged);
    }

    public void update() {
        updatedFromOdometry = false;
        CellCluster target = targetSupplier.get();
        if (!drive.isFieldPoseInitialized() || target == null) return;

        Drive.Pose pose = drive.getOdometryPose();
        double measuredDistanceCm = Math.hypot(
                target.getTargetXmm() - pose.xMm,
                target.getTargetYmm() - pose.yMm) / 10.0;
        if (!Double.isFinite(measuredDistanceCm)) return;

        ShotLookupTable.Setpoint raw = calibration.lookup(measuredDistanceCm).orElse(null);
        if (raw == null) return;

        distanceCm = measuredDistanceCm;
        setpoint = new ShotLookupTable.Setpoint(
                measuredDistanceCm,
                Range.clip(raw.targetSpeed, 0.0, ShooterConstants.speedMax),
                Range.clip(raw.pitchPosition,
                        ShooterConstants.pitchMinPosition,
                        ShooterConstants.pitchMaxPosition));
        updatedFromOdometry = true;
    }

    /** Last valid result is retained if field odometry is temporarily unavailable. */
    public Optional<ShotLookupTable.Setpoint> getSetpoint() {
        return Optional.ofNullable(setpoint);
    }

    public double getDistanceCm() {
        return distanceCm;
    }

    public boolean wasUpdatedFromOdometryThisLoop() {
        return updatedFromOdometry;
    }

    public int getCalibrationPointCount() {
        return calibration.size();
    }
}
