package org.firstinspires.ftc.teamcode.subsystems.feeder;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Feeder extends SubsystemBase {
    private final DcMotor intakeMotor;
    private final DcMotor transitMotor;
    private final Servo limitServo;
    private boolean limitOpen;
    private double intakePower;
    private double transitPower;

    public Feeder(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, FeederConstants.intakeMotorName);
        transitMotor = hardwareMap.get(DcMotor.class, FeederConstants.transitMotorName);
        limitServo = hardwareMap.get(Servo.class, FeederConstants.limitServoName);

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transitMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transitMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stop();
    }

    public void setOutputs(double intake, double transit, boolean openLimit) {
        intakePower = clip(intake);
        transitPower = clip(transit);
        limitOpen = openLimit;
        intakeMotor.setPower(intakePower);
        transitMotor.setPower(transitPower);
        limitServo.setPosition(limitOpen
                ? FeederConstants.limitOpen : FeederConstants.limitClosed);
    }

    public void stop() {
        setOutputs(0.0, 0.0, false);
    }

    public boolean isLimitOpen() {
        return limitOpen;
    }

    public double getIntakePower() {
        return intakePower;
    }

    public double getTransitPower() {
        return transitPower;
    }

    private static double clip(double value) {
        return Double.isFinite(value) ? Math.max(-1.0, Math.min(1.0, value)) : 0.0;
    }
}
