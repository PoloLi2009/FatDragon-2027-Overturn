package org.firstinspires.ftc.teamcode.subsystems.vision;

public class VisionConstants {
    public static String limelightName = "limelight";
    public static int bluePipelineIndex = 0;
    public static int redPipelineIndex = 1;
    public static int pollRateHz = 50;
    public static long maxResultAgeMs = 200;
    // Minimum individual AprilTag area as a percentage of the image (0-100).
    public static double minimumTargetArea = 0.01;
    public static int minimumBotposeTagCount = 2;

    // Field center origin; +X toward audience, +Y toward blue HIVE.
    public static double hivePivotHeightInches = 43.95;
    public static double maximumHiveTiltDegrees = 35.0;
    public static double robotLevelToleranceDegrees = 10.0;


    private VisionConstants() {
    }
}
