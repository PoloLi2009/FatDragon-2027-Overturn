package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.vision.CellCluster;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;

public class DriveAimCommand extends CommandBase {
    private final Drive drive;
    private final Vision vision;
    private final Gamepad gamepad;
    private final PIDController aimPid = new PIDController(
            DriveConstants.rearAimKp, DriveConstants.rearAimKi, DriveConstants.rearAimKd);

    private double lastAppliedFrameTimestamp = Double.NaN;
    private CellCluster target;
    private CellCluster previousTarget;
    private double aimErrorRadians = Double.NaN;
    private double turnPower;
    private boolean visionApplied;
    private boolean aligned;

    public DriveAimCommand(Drive drive, Vision vision, Gamepad gamepad) {
        this.drive = drive;
        this.vision = vision;
        this.gamepad = gamepad;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        drive.updateOdometry();
        applyLatestVisionPose();

        Drive.Pose pose = drive.getOdometryPose();
        target = drive.isFieldPoseInitialized()
                ? CellCluster.forFieldX(vision.getAlliance(), pose.xMm).orElse(null)
                : null;
        aimErrorRadians = target == null ? Double.NaN
                : target.rearHeadingErrorRadians(pose.xMm, pose.yMm, pose.headingRadians);

        boolean aimRequested = gamepad.right_trigger > 0.1;
        boolean aimReady = Double.isFinite(aimErrorRadians);
        double absoluteErrorDegrees = Math.abs(Math.toDegrees(aimErrorRadians));
        if (!aimRequested || !aimReady || target != previousTarget) {
            aligned = false;
        } else if (aligned) {
            // Hysteresis prevents repeatedly restarting across the 3.5-degree boundary.
            aligned = absoluteErrorDegrees <= DriveConstants.rearAimExitToleranceDegrees;
        } else {
            aligned = absoluteErrorDegrees <= DriveConstants.rearAimToleranceDegrees;
        }

        double forward = Drive.deadband(-gamepad.left_stick_y);
        double strafe = Drive.deadband(gamepad.left_stick_x);
        turnPower = Drive.deadband(gamepad.right_stick_x);

        aimPid.setPID(DriveConstants.rearAimKp,
                DriveConstants.rearAimKi, DriveConstants.rearAimKd);
        if (!aimRequested || !aimReady || aligned || target != previousTarget) {
            aimPid.reset();
        }
        previousTarget = target;
        if (aimRequested) {
            // Mecanum positive turn is clockwise; field-heading error is CCW positive.
            turnPower = aimReady && !aligned ? Drive.shapeRearAimTurn(
                    -aimPid.calculate(-aimErrorRadians, 0.0),
                    aimErrorRadians) : 0.0;
        }
        drive.driveFieldCentric(forward, strafe, turnPower);
    }

    @Override
    public void end(boolean interrupted) {
        aimPid.reset();
        drive.stop();
    }

    private void applyLatestVisionPose() {
        visionApplied = false;
        Pose2D correctedPose = vision.getCorrectedRobotFieldPose().orElse(null);
        double frameTimestamp = vision.getFrameTimestamp();
        if (correctedPose != null && Double.isFinite(frameTimestamp)
                && (frameTimestamp != lastAppliedFrameTimestamp
                    || !drive.isFieldPoseInitialized())) {
            visionApplied = drive.applyLimelightPose(correctedPose);
            if (visionApplied) {
                lastAppliedFrameTimestamp = frameTimestamp;
            }
        }
    }

    public CellCluster getTarget() {
        return target;
    }

    public double getAimErrorDegrees() {
        return Math.toDegrees(aimErrorRadians);
    }

    public double getTurnPower() {
        return turnPower;
    }

    public boolean isAligned() {
        return aligned;
    }

    public boolean wasVisionAppliedThisLoop() {
        return visionApplied;
    }

}
