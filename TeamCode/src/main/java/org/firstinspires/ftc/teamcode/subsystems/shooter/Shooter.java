package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class Shooter extends SubsystemBase {
    private final DcMotorEx leftMotor;
    private final DcMotorEx rightMotor;
    private final Servo pitchServo;
    private double pitchPosition = ShooterConstants.pitchMinPosition;
    private double requestedSpeed;
    private boolean powerTakeoverActive;
    private double takeoverPower;

    public Shooter(HardwareMap hardwareMap) {
        leftMotor = hardwareMap.get(DcMotorEx.class, ShooterConstants.leftShooterMotorName);
        rightMotor = hardwareMap.get(DcMotorEx.class, ShooterConstants.rightShooterMotorName);
        pitchServo = hardwareMap.get(Servo.class, ShooterConstants.pitchServoName);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        applyVelocityPidf();

        setPitchPosition(pitchPosition);
        stop();
    }

    public void setVelocity(double ticksPerSecond) {
        setVelocity(ticksPerSecond, false);
    }

    public void setVelocity(double ticksPerSecond, boolean allowPowerTakeover) {
        requestedSpeed = Range.clip(ticksPerSecond, 0.0, ShooterConstants.speedMax);
        if (requestedSpeed <= 0.0) {
            stop();
            return;
        }

        double deficitFraction = (requestedSpeed - getMinimumVelocity()) / requestedSpeed;
        if (!allowPowerTakeover || !Double.isFinite(deficitFraction)) {
            powerTakeoverActive = false;
        } else if (powerTakeoverActive) {
            powerTakeoverActive = deficitFraction
                    > ShooterConstants.powerTakeoverReleaseDeficitFraction;
        } else {
            powerTakeoverActive = deficitFraction
                    >= ShooterConstants.powerTakeoverStartDeficitFraction;
        }

        if (powerTakeoverActive) {
            ensurePowerMode();
            takeoverPower = Range.clip(
                    ShooterConstants.powerTakeoverMaximumPower, 0.0, 1.0);
            leftMotor.setPower(-takeoverPower);
            rightMotor.setPower(takeoverPower);
        } else {
            ensureVelocityMode();
            takeoverPower = 0.0;
            leftMotor.setVelocity(-requestedSpeed);
            rightMotor.setVelocity(requestedSpeed);
        }
    }

    public void idle() {
        setVelocity(ShooterConstants.idleSpeed);
    }

    public void stop() {
        requestedSpeed = 0.0;
        powerTakeoverActive = false;
        takeoverPower = 0.0;
        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);
    }

    public void setPitchPosition(double position) {
        pitchPosition = Range.clip(position,
                ShooterConstants.pitchMinPosition, ShooterConstants.pitchMaxPosition);
        pitchServo.setPosition(pitchPosition);
    }

    public double getLeftVelocity() {
        return Math.abs(leftMotor.getVelocity());
    }

    public double getRightVelocity() {
        return Math.abs(rightMotor.getVelocity());
    }

    public double getMinimumVelocity() {
        return Math.min(getLeftVelocity(), getRightVelocity());
    }

    public double getRequestedSpeed() {
        return requestedSpeed;
    }

    public boolean isPowerTakeoverActive() {
        return powerTakeoverActive;
    }

    public double getTakeoverPower() {
        return takeoverPower;
    }

    public double getPitchPosition() {
        return pitchPosition;
    }

    public double getTicksPerRevolution() {
        double ticks = leftMotor.getMotorType().getTicksPerRev();
        return ticks == 0.0 ? 28.0 : ticks;
    }

    private void ensurePowerMode() {
        if (leftMotor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
            leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    private void ensureVelocityMode() {
        if (leftMotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            applyVelocityPidf();
        }
    }

    private void applyVelocityPidf() {
        leftMotor.setVelocityPIDFCoefficients(
                ShooterConstants.kP, ShooterConstants.kI,
                ShooterConstants.kD, ShooterConstants.kF);
        rightMotor.setVelocityPIDFCoefficients(
                ShooterConstants.kP, ShooterConstants.kI,
                ShooterConstants.kD, ShooterConstants.kF);
    }
}
