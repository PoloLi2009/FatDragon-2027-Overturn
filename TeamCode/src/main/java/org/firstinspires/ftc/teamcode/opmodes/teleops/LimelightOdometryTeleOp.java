package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.vision.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.vision.CellCluster;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;

import java.util.Locale;

/** Bring-up TeleOp for field-centric mecanum drive, odometry, and Limelight. */
@TeleOp(name = "Test: Limelight + Odometry", group = "Test")
public class LimelightOdometryTeleOp extends CommandOpMode {
    private Drive drive;
    private Vision vision;
    private boolean previousX;
    private double lastAppliedFrameTimestamp = Double.NaN;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final PIDController rearAimPid = new PIDController(
            DriveConstants.rearAimKp, DriveConstants.rearAimKi, DriveConstants.rearAimKd);
    private CellCluster previousAimTarget;
    private boolean rearAimAligned;

    @Override
    public void initialize() {
        drive = new Drive(hardwareMap);
        vision = new Vision(hardwareMap, Alliance.BLUE);

        telemetry.addLine("Limelight + Odometry test ready");
        telemetry.addLine("X: switch BLUE/RED | Hold RIGHT trigger: rear aims at Cell");
        telemetry.addLine("Valid new vision poses automatically replace OD; no averaging");
        telemetry.update();
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        drive.updateOdometry();

        if (gamepad1.x && !previousX) {
            vision.setAlliance(vision.getAlliance() == Alliance.BLUE
                    ? Alliance.RED
                    : Alliance.BLUE);
            lastAppliedFrameTimestamp = Double.NaN;
        }
        previousX = gamepad1.x;

        Pose3D rawPose = vision.getRawRobotFieldPose().orElse(null);
        Pose2D correctedPose = vision.getCorrectedRobotFieldPose().orElse(null);
        boolean rawLimelightPoseValid = rawPose != null;
        boolean correctedLimelightPoseValid = correctedPose != null;

        // Do not repeatedly reset moving odometry to the same physical camera frame.
        boolean visionApplied = false;
        double frameTimestamp = vision.getFrameTimestamp();
        if (correctedLimelightPoseValid && Double.isFinite(frameTimestamp)
                && (frameTimestamp != lastAppliedFrameTimestamp
                    || !drive.isFieldPoseInitialized())) {
            visionApplied = drive.applyLimelightPose(correctedPose);
            if (visionApplied) lastAppliedFrameTimestamp = frameTimestamp;
        }

        Drive.Pose odometryPose = drive.getOdometryPose();
        CellCluster target = drive.isFieldPoseInitialized()
                ? CellCluster.forFieldX(vision.getAlliance(), odometryPose.xMm).orElse(null)
                : null;
        double aimError = target == null ? Double.NaN : target.rearHeadingErrorRadians(
                odometryPose.xMm, odometryPose.yMm, odometryPose.headingRadians);
        boolean aimRequested = gamepad1.right_trigger > 0.1;
        boolean aimReady = Double.isFinite(aimError);
        double absoluteErrorDegrees = Math.abs(Math.toDegrees(aimError));
        if (!aimRequested || !aimReady || target != previousAimTarget) {
            rearAimAligned = false;
        } else if (rearAimAligned) {
            rearAimAligned = absoluteErrorDegrees
                    <= DriveConstants.rearAimExitToleranceDegrees;
        } else {
            rearAimAligned = absoluteErrorDegrees
                    <= DriveConstants.rearAimToleranceDegrees;
        }
        boolean aligned = rearAimAligned;
        double forward = Drive.deadband(-gamepad1.left_stick_y);
        double strafe = Drive.deadband(gamepad1.left_stick_x);
        double turn = Drive.deadband(gamepad1.right_stick_x);
        rearAimPid.setPID(DriveConstants.rearAimKp,
                DriveConstants.rearAimKi, DriveConstants.rearAimKd);
        if (!aimRequested || !aimReady || aligned || target != previousAimTarget) {
            rearAimPid.reset();
        }
        previousAimTarget = target;
        if (aimRequested) {
            // Clockwise motor command is the opposite sign of CCW heading error.
            // Error is in radians; output is clipped after PID calculation.
            turn = aimReady && !aligned ? Drive.shapeRearAimTurn(
                    -rearAimPid.calculate(-aimError, 0.0), aimError) : 0.0;
        }
        drive.driveFieldCentric(forward, strafe, turn);

        telemetry.addData("Alliance", vision.getAlliance());
        telemetry.addData("Pipeline", vision.getExpectedPipelineIndex());
        telemetry.addData("Pinpoint", drive.getPinpointStatus());
        telemetry.addData("OD coordinate frame", drive.isFieldPoseInitialized()
                ? "FIELD: +X audience, +Y blue"
                : "LOCAL: waiting for valid vision to align position AND heading");
        telemetry.addData("Limelight raw pose valid", rawLimelightPoseValid);
        telemetry.addData("Limelight raw pose", rawLimelightPoseValid
                ? formatPose(rawPose) : "unavailable");
        telemetry.addData("Hive-corrected pose valid", correctedLimelightPoseValid);
        telemetry.addData("Hive correction", "%.1f° (level error %.1f°)",
                vision.getHiveAngleDegrees(),
                vision.getRobotLevelErrorDegrees());
        telemetry.addData("Hive-corrected pose", correctedLimelightPoseValid
                ? formatPose(correctedPose) : "unavailable");
        telemetry.addData("Odometry pose", formatPose(odometryPose));
        telemetry.addData("Vision status", vision.getPoseStatus());
        telemetry.addData("Actual pipeline", vision.getActualPipelineIndex());
        telemetry.addData("Visible tags", vision.getVisibleTagIds());
        telemetry.addLine("Field center (0,0); +X audience; +Y blue HIVE");
        telemetry.addData("Vision directly applied this loop", visionApplied);
        telemetry.addData("Target Cell", target == null ? "unavailable: no field pose" : target);
        if (target != null) {
            telemetry.addData("Target (mm)", "x=%.1f y=%.1f",
                    target.getTargetXmm(), target.getTargetYmm());
        }
        telemetry.addData("Rear aim error (deg)", "%.1f", Math.toDegrees(aimError));
        telemetry.addData("Rear aim", !aimRequested ? "MANUAL"
                : !aimReady ? "UNAVAILABLE: no usable field pose/target"
                : aligned ? "ALIGNED" : "TURNING");
        telemetry.addData("Turn power", "%.2f", turn);
        telemetry.addData("Loop time (ms)", "%.1f", loopTimer.milliseconds());
        telemetry.addLine("X = BLUE/RED | Hold RIGHT trigger = rear aim | Vision -> OD automatically");
        telemetry.update();
        loopTimer.reset();
    }

    private String formatPose(Pose3D pose) {
        Position position = pose.getPosition().toUnit(DistanceUnit.MM);
        double headingDegrees = pose.getOrientation().getYaw(AngleUnit.DEGREES);
        return String.format(Locale.US, "x=%.0f y=%.0f h=%.1f° mm",
                position.x, position.y, headingDegrees);
    }

    private String formatPose(Pose2D pose) {
        return String.format(Locale.US, "x=%.0f y=%.0f h=%.1f° mm",
                pose.getX(DistanceUnit.MM),
                pose.getY(DistanceUnit.MM),
                pose.getHeading(AngleUnit.DEGREES));
    }

    private String formatPose(Drive.Pose pose) {
        return String.format(Locale.US, "x=%.0f y=%.0f h=%.1f° mm",
                pose.xMm, pose.yMm, Math.toDegrees(pose.headingRadians));
    }
}
