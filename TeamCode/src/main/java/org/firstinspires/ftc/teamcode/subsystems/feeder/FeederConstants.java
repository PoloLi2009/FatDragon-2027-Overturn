package org.firstinspires.ftc.teamcode.subsystems.feeder;

/** Values carried over unchanged from the tuned Shooter Test PID OpMode. */
public final class FeederConstants {
    private FeederConstants() { }

    public static String intakeMotorName = "intakeMotor";
    public static String transitMotorName = "transitMotor";
    public static String limitServoName = "limitServo";

    public static double limitClosed = 0.4;
    public static double limitOpen = 0.7;
    public static double intakeSign = 1.0;
    public static double transitIntakeSign = -1.0;
    public static double transitFireSign = -1.0;
    public static double fireIntakePowerScale = 0.75;
    public static double fireTransitPowerScale = 0.60;
    public static long retractTimeMs = 10;
}
