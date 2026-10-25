package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.Trigger;

import org.firstinspires.ftc.teamcode.commands.DriveAimCommand;
import org.firstinspires.ftc.teamcode.commands.LauncherCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.feeder.Feeder;
import org.firstinspires.ftc.teamcode.subsystems.feeder.FeederConstants;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShotCalibrationStore;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShotLookupTable;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.vision.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.vision.CellCluster;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;
import org.firstinspires.ftc.teamcode.tasks.DistanceShotTaskHook;

import java.util.Locale;

public abstract class TeleOpBase extends CommandOpMode {
    private Drive drive;
    private Vision vision;
    private Shooter shooter;
    private Feeder feeder;
    private DriveAimCommand driveCommand;
    private LauncherCommand launcherCommand;
    private DistanceShotTaskHook distanceShotTaskHook;
    private ShotCalibrationStore shotCalibrationStore;
    private String calibrationStatus = "No point recorded this run";
    private int calibrationPointCount;

    protected abstract Alliance getAlliance();

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        drive = new Drive(hardwareMap);
        vision = new Vision(hardwareMap, getAlliance());
        shooter = new Shooter(hardwareMap);
        feeder = new Feeder(hardwareMap);
        shotCalibrationStore = new ShotCalibrationStore();
        try {
            if (shotCalibrationStore.clearExistingDataOnce()) {
                calibrationStatus = "Previous accidental calibration data cleared";
            }
            boolean restoredFarPoint = shotCalibrationStore.ensurePoint(
                    Alliance.RED, CellCluster.RED_AUDIENCE,
                    113.144, 1650.0, 0.1600);
            boolean addedNearPoint = shotCalibrationStore.ensurePoint(
                    Alliance.RED, CellCluster.RED_AUDIENCE,
                    70.0, 1600.0, 0.4000);
            if (restoredFarPoint || addedNearPoint) {
                calibrationStatus = "Required 70 cm and 113.144 cm points synchronized";
            }
        } catch (Exception error) {
            calibrationStatus = "CLEAR FAILED: " + error.getClass().getSimpleName()
                    + ": " + error.getMessage();
        }
        refreshCalibrationPointCount();

        driveCommand = new DriveAimCommand(drive, vision, gamepad1);
        distanceShotTaskHook = new DistanceShotTaskHook(
                drive, driveCommand::getTarget, shotCalibrationStore.getFile());
        launcherCommand = new LauncherCommand(
                shooter, feeder, gamepad1, distanceShotTaskHook);
        drive.setDefaultCommand(driveCommand);
        shooter.setDefaultCommand(launcherCommand);

        // Only the driver's field-centric reference changes; Pinpoint's field pose is untouched.
        new Trigger(() -> gamepad1.left_stick_button)
                .whenActive(drive::resetDriverHeading);
        new Trigger(() -> gamepad1.a)
                .whenActive(this::recordShotCalibration);

        telemetry.addData("Alliance", getAlliance());
        telemetry.addLine("Left stick button: re-zero driver heading only");
        telemetry.addLine("RT: rear auto-aim + shoot | LT: intake");
        telemetry.addLine("A: record distance / shooter speed / pitch to CSV");
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();
        showTelemetry();
    }

    private void showTelemetry() {
        Drive.Pose pose = drive.getOdometryPose();
        CellCluster target = driveCommand.getTarget();
        double ticksPerRev = shooter.getTicksPerRevolution();
        telemetry.addData("Alliance / Pipeline", "%s / expected %d / actual %d",
                getAlliance(), vision.getExpectedPipelineIndex(), vision.getActualPipelineIndex());
        telemetry.addData("Vision", vision.getPoseStatus());
        telemetry.addData("Vision -> OD this loop", driveCommand.wasVisionAppliedThisLoop());
        telemetry.addData("Field pose", "x=%.0f y=%.0f h=%.1f° mm (%s)",
                pose.xMm, pose.yMm, Math.toDegrees(pose.headingRadians),
                drive.isFieldPoseInitialized() ? "FIELD" : "WAITING FOR VISION");
        telemetry.addData("Target Cell", target == null ? "unavailable" : target);
        if (target != null) {
            telemetry.addData("Target position", "x=%.1f y=%.1f mm",
                    target.getTargetXmm(), target.getTargetYmm());
        }
        telemetry.addData("Rear aim", gamepad1.right_trigger <= 0.1 ? "MANUAL"
                : target == null ? "NO FIELD TARGET"
                : driveCommand.isAligned() ? "ALIGNED" : "TURNING");
        telemetry.addData("Aim error / turn", "%.1f° / %.2f",
                driveCommand.getAimErrorDegrees(), driveCommand.getTurnPower());

        telemetry.addData("Shooter target", "%.0f ticks/s (%.0f RPM)",
                launcherCommand.getTargetSpeed(),
                launcherCommand.getTargetSpeed() * 60.0 / ticksPerRev);
        telemetry.addData("Shooter L / R", "%.0f / %.0f ticks/s",
                shooter.getLeftVelocity(), shooter.getRightVelocity());
        telemetry.addData("Auto shot", "%s | %.1f cm | %s",
                launcherCommand.isAutomaticSetpointActive() ? "ACTIVE" : "WAITING FOR FIELD OD",
                distanceShotTaskHook.getDistanceCm(),
                distanceShotTaskHook.wasUpdatedFromOdometryThisLoop()
                        ? "CURRENT OD" : "LAST VALID OD");
        telemetry.addData("Auto curve / offsets", "%d points | speed %+.0f | pitch %+.3f",
                distanceShotTaskHook.getCalibrationPointCount(),
                launcherCommand.getSpeedOffset(), launcherCommand.getPitchOffset());
        telemetry.addData("Shooter control", shooter.isPowerTakeoverActive()
                ? String.format(Locale.US, "POWER TAKEOVER %.2f", shooter.getTakeoverPower())
                : "VELOCITY PIDF");
        telemetry.addData("Takeover delay", "%d ms (%s, elapsed %d ms)",
                launcherCommand.getPowerTakeoverDelayMs(),
                launcherCommand.isPowerTakeoverArmed() ? "ARMED" : "WAITING",
                launcherCommand.getFiringElapsedMs());
        telemetry.addData("Speed stable", "%s (%d / %d ms)",
                launcherCommand.isStable() ? "YES"
                        : launcherCommand.isSpeedInTolerance() ? "TIMING" : "NO",
                launcherCommand.getStableElapsedMs(), ShooterConstants.stableTimeMs);
        telemetry.addData("Firing / Gate", "%s / %s",
                launcherCommand.isFiring() ? "FIRING" : "IDLE",
                feeder.isLimitOpen() ? "OPEN" : "CLOSED");
        telemetry.addData("Transit", launcherCommand.isRetracting()
                ? "RETRACTING" : String.format(Locale.US, "%.2f", feeder.getTransitPower()));
        telemetry.addData("Fire intake / transit", "%.2f / %.2f",
                FeederConstants.fireIntakePowerScale,
                FeederConstants.fireTransitPowerScale);
        telemetry.addData("Pitch", "%.2f", shooter.getPitchPosition());
        telemetry.addData("Shot calibration", "%s (table points: %d)",
                calibrationStatus, calibrationPointCount);
        telemetry.addData("Calibration CSV", shotCalibrationStore.getFile().getAbsolutePath());
        telemetry.addLine("D-pad L/R: speed -/+ | D-pad D/U: pitch -/+");
        telemetry.addLine("Y/B: takeover delay +10/-10 ms");
        telemetry.addLine("LB: reverse intake only | RB: reverse intake + transit");
        telemetry.addLine("A: save current distance, preset speed, and pitch");
        telemetry.addLine("Left stick button resets driving head only; field pose is preserved");
        telemetry.update();
    }

    private void recordShotCalibration() {
        CellCluster target = driveCommand.getTarget();
        if (!drive.isFieldPoseInitialized() || target == null) {
            calibrationStatus = "NOT SAVED: waiting for valid field pose/Cell";
            return;
        }
        Drive.Pose pose = drive.getOdometryPose();
        double distanceCm = Math.hypot(
                target.getTargetXmm() - pose.xMm,
                target.getTargetYmm() - pose.yMm) / 10.0;
        try {
            shotCalibrationStore.append(
                    getAlliance(), target, distanceCm,
                    launcherCommand.getTargetSpeed(), shooter.getPitchPosition());
            refreshCalibrationPointCount();
            distanceShotTaskHook.reloadCalibration();
            calibrationStatus = String.format(Locale.US,
                    "SAVED: %.1f cm / %.0f ticks/s / pitch %.3f",
                    distanceCm, launcherCommand.getTargetSpeed(), shooter.getPitchPosition());
        } catch (Exception error) {
            calibrationStatus = "SAVE FAILED: " + error.getClass().getSimpleName()
                    + ": " + error.getMessage();
        }
    }

    private void refreshCalibrationPointCount() {
        try {
            calibrationPointCount = ShotLookupTable
                    .load(shotCalibrationStore.getFile()).size();
        } catch (Exception error) {
            calibrationPointCount = 0;
            calibrationStatus = "TABLE READ FAILED: " + error.getClass().getSimpleName();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            super.runOpMode();
        } finally {
            reset();
            if (drive != null) drive.stop();
            if (shooter != null) shooter.stop();
            if (feeder != null) feeder.stop();
            if (vision != null) vision.stop();
        }
    }
}
