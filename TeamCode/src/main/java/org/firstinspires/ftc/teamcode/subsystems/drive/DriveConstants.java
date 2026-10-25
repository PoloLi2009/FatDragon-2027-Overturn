package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/** Hardware configuration copied from the FTC-16095 FatDragon 2027 public project. */
public final class DriveConstants {
    private DriveConstants() { }

    public static String leftFrontMotorName = "leftFrontMotor";
    public static String leftBackMotorName = "leftBackMotor";
    public static String rightFrontMotorName = "rightFrontMotor";
    public static String rightBackMotorName = "rightBackMotor";

    public static DcMotorSimple.Direction leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction leftBackMotorDirection = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction rightBackMotorDirection = DcMotorSimple.Direction.FORWARD;
    public static double joystickDeadband = 0.03;
    public static double teleOpScale = 1.0;

    public static String pinpointName = "pinpoint";
    public static GoBildaPinpointDriver.GoBildaOdometryPods pinpointPodType =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
    public static double pinpointXOffsetMm = -96.0;
    public static double pinpointYOffsetMm = -74.0;
    // X/forward pod reversed following the robot's direction check.
    public static GoBildaPinpointDriver.EncoderDirection pinpointForwardPodDirection =
            GoBildaPinpointDriver.EncoderDirection.REVERSED;
    public static GoBildaPinpointDriver.EncoderDirection pinpointStrafePodDirection =
            GoBildaPinpointDriver.EncoderDirection.REVERSED;

    // Positive mecanum turn is clockwise; field heading is counterclockwise.
    public static double rearAimKp = 0.35;
    // I remains disabled. Keep D modest because field-pose corrections contain measurement noise.
    public static double rearAimKi = 0.0;
    public static double rearAimKd = 0.01;
    public static double rearAimMaxPower = 0.65;
    public static double rearAimMinimumPower = 0.12;
    public static double rearAimMinimumPowerZoneDegrees = 20.0;
    public static double rearAimToleranceDegrees = 3.5;
    public static double rearAimExitToleranceDegrees = 5.5;
}
