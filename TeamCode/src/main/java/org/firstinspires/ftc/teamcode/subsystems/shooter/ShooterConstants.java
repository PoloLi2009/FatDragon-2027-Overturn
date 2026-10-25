package org.firstinspires.ftc.teamcode.subsystems.shooter;

/** Values carried over unchanged from the tuned Shooter Test PID OpMode. */
public final class ShooterConstants {
    private ShooterConstants() { }

    public static String leftShooterMotorName = "leftShooterMotor";
    public static String rightShooterMotorName = "rightShooterMotor";
    public static String pitchServoName = "pitchServo";

    public static double pitchMinPosition = 0.0;
    public static double pitchMaxPosition = 0.55;
    public static double pitchStep = 0.02;

    public static double initialTargetSpeed = 1800.0;
    public static double speedStep = 50.0;
    public static double speedMax = 2500.0;
    public static double speedToleranceFraction = 0.075;
    public static double idleSpeed = 900.0;
    public static long stableTimeMs = 500;
    public static long fireLockoutMs = 200;

    public static double kP = 20.0;
    public static double kI = 0.20;
    public static double kD = 0.65;
    public static double kF = 13.5;

    // Once firing, any meaningful drop switches immediately to full open-loop power.
    public static long powerTakeoverDelayMs = 150;
    public static long powerTakeoverDelayStepMs = 10;
    public static long powerTakeoverDelayMinMs = 0;
    public static long powerTakeoverDelayMaxMs = 500;
    public static double powerTakeoverStartDeficitFraction = 0.010;
    public static double powerTakeoverReleaseDeficitFraction = 0.0;
    public static double powerTakeoverMaximumPower = 1.00;
}
