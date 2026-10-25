package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/** Minimal field-centric mecanum chassis using GoBilda Pinpoint Odometry. */
public class Drive extends SubsystemBase {
    private final DcMotorEx leftFrontMotor;
    private final DcMotorEx leftBackMotor;
    private final DcMotorEx rightFrontMotor;
    private final DcMotorEx rightBackMotor;
    private final GoBildaPinpointDriver pinpoint;
    private double driverHeadingReferenceRadians = Double.NaN;
    private boolean fieldPoseInitialized;

    public boolean isFieldPoseInitialized() {
        return fieldPoseInitialized;
    }

    public Drive(HardwareMap hardwareMap) {
        leftFrontMotor = hardwareMap.get(DcMotorEx.class, DriveConstants.leftFrontMotorName);
        leftBackMotor = hardwareMap.get(DcMotorEx.class, DriveConstants.leftBackMotorName);
        rightFrontMotor = hardwareMap.get(DcMotorEx.class, DriveConstants.rightFrontMotorName);
        rightBackMotor = hardwareMap.get(DcMotorEx.class, DriveConstants.rightBackMotorName);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, DriveConstants.pinpointName);

        leftFrontMotor.setDirection(DriveConstants.leftFrontMotorDirection);
        leftBackMotor.setDirection(DriveConstants.leftBackMotorDirection);
        rightFrontMotor.setDirection(DriveConstants.rightFrontMotorDirection);
        rightBackMotor.setDirection(DriveConstants.rightBackMotorDirection);

        setBrakeMode();
        pinpoint.setEncoderResolution(DriveConstants.pinpointPodType);
        pinpoint.setEncoderDirections(
                DriveConstants.pinpointForwardPodDirection,
                DriveConstants.pinpointStrafePodDirection);
        pinpoint.setOffsets(
                DriveConstants.pinpointXOffsetMm,
                DriveConstants.pinpointYOffsetMm,
                DistanceUnit.MM);
        pinpoint.resetPosAndIMU();
        stop();
    }

    /** Refresh Pinpoint and establish the driver-relative heading reference. */
    public void updateOdometry() {
        pinpoint.update();
        if (pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) {
            fieldPoseInitialized = false;
            driverHeadingReferenceRadians = Double.NaN;
        }
        if (pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY
                && !Double.isFinite(driverHeadingReferenceRadians)) {
            resetDriverHeading();
        }
    }

    public void driveFieldCentric(double forward, double strafe, double turn) {
        double heading = getHeadingRadians();
        if (!Double.isFinite(heading)) {
            stop();
            return;
        }

        double robotForward = forward * Math.cos(heading) - strafe * Math.sin(heading);
        double robotStrafe = forward * Math.sin(heading) + strafe * Math.cos(heading);
        driveRobotCentric(robotForward, robotStrafe, turn);
    }

    private void driveRobotCentric(double forward, double strafe, double turn) {
        double denominator = Math.max(
                Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1.0);
        double leftFrontPower = (forward + strafe + turn) / denominator;
        double leftBackPower = (forward - strafe + turn) / denominator;
        double rightFrontPower = (forward - strafe - turn) / denominator;
        double rightBackPower = (forward + strafe - turn) / denominator;

        leftFrontMotor.setPower(leftFrontPower * DriveConstants.teleOpScale);
        leftBackMotor.setPower(leftBackPower * DriveConstants.teleOpScale);
        rightFrontMotor.setPower(rightFrontPower * DriveConstants.teleOpScale);
        rightBackMotor.setPower(rightBackPower * DriveConstants.teleOpScale);
    }

    public void stop() {
        leftFrontMotor.setPower(0.0);
        leftBackMotor.setPower(0.0);
        rightFrontMotor.setPower(0.0);
        rightBackMotor.setPower(0.0);
    }

    /** Directly replace translation AND heading with the valid corrected vision pose. */
    public boolean applyLimelightPose(Pose2D limelightPose) {
        if (limelightPose == null
                || pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY
                || !Double.isFinite(limelightPose.getX(DistanceUnit.MM))
                || !Double.isFinite(limelightPose.getY(DistanceUnit.MM))
                || !Double.isFinite(limelightPose.getHeading(AngleUnit.RADIANS))) return false;

        setPinpointPose(limelightPose);
        fieldPoseInitialized = pinpoint.getDeviceStatus()
                == GoBildaPinpointDriver.DeviceStatus.READY;
        return fieldPoseInitialized;
    }

    public Pose getOdometryPose() {
        Pose2D pose = pinpoint.getPosition();
        return new Pose(
                pose.getX(DistanceUnit.MM),
                pose.getY(DistanceUnit.MM),
                pose.getHeading(AngleUnit.RADIANS));
    }

    public GoBildaPinpointDriver.DeviceStatus getPinpointStatus() {
        return pinpoint.getDeviceStatus();
    }

    public double getHeadingRadians() {
        if (pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY
                || !Double.isFinite(driverHeadingReferenceRadians)) {
            return Double.NaN;
        }
        return normalizeAngle(
                pinpoint.getHeading(AngleUnit.RADIANS) - driverHeadingReferenceRadians);
    }

    public void resetDriverHeading() {
        driverHeadingReferenceRadians = pinpoint.getHeading(AngleUnit.RADIANS);
    }

    public static double deadband(double value) {
        return Math.abs(value) < DriveConstants.joystickDeadband ? 0.0 : value;
    }

    /**
     * Preserve proportional high-speed turning at large errors, but overcome drivetrain
     * friction with a fixed low output inside the final 20-degree approach.
     */
    public static double shapeRearAimTurn(double rawPidTurn, double headingErrorRadians) {
        if (!Double.isFinite(rawPidTurn) || !Double.isFinite(headingErrorRadians)
                || headingErrorRadians == 0.0) return 0.0;
        double magnitude = Math.min(
                Math.abs(rawPidTurn), DriveConstants.rearAimMaxPower);
        if (Math.abs(Math.toDegrees(headingErrorRadians))
                <= DriveConstants.rearAimMinimumPowerZoneDegrees) {
            magnitude = Math.max(magnitude, DriveConstants.rearAimMinimumPower);
        }
        // Positive field-heading error requires a clockwise (negative) motor turn.
        return Math.copySign(magnitude, -headingErrorRadians);
    }

    private void setPinpointPose(Pose2D pose) {
        double previousHeading = pinpoint.getHeading(AngleUnit.RADIANS);
        pinpoint.setPosition(pose);
        pinpoint.update();

        if (Double.isFinite(driverHeadingReferenceRadians)
                && Double.isFinite(previousHeading)) {
            double correction = Math.IEEEremainder(
                    pose.getHeading(AngleUnit.RADIANS) - previousHeading,
                    2.0 * Math.PI);
            driverHeadingReferenceRadians = normalizeAngle(
                    driverHeadingReferenceRadians + correction);
        } else {
            resetDriverHeading();
        }
    }

    private void setBrakeMode() {
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private static double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }

    public static final class Pose {
        public double xMm;
        public double yMm;
        public double headingRadians;

        public Pose(double xMm, double yMm, double headingRadians) {
            this.xMm = xMm;
            this.yMm = yMm;
            this.headingRadians = headingRadians;
        }
    }
}
