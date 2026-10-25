package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.feeder.Feeder;
import org.firstinspires.ftc.teamcode.subsystems.feeder.FeederConstants;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShotLookupTable;
import org.firstinspires.ftc.teamcode.tasks.DistanceShotTaskHook;

public class LauncherCommand extends CommandBase {
    private final Shooter shooter;
    private final Feeder feeder;
    private final Gamepad gamepad;
    private final DistanceShotTaskHook distanceShotTaskHook;

    private double targetSpeed = ShooterConstants.initialTargetSpeed;
    private double pitchPosition = ShooterConstants.pitchMinPosition;
    private boolean previousUp;
    private boolean previousDown;
    private boolean previousLeft;
    private boolean previousRight;
    private boolean previousY;
    private boolean previousB;
    private double previousRightTrigger;
    private long rightTriggerPressedAtMs;
    private long stableStartedAtMs;
    private long fireStartedAtMs;
    private boolean stable;
    private boolean firing;
    private boolean retracting;
    private boolean speedInTolerance;
    private long powerTakeoverDelayMs = ShooterConstants.powerTakeoverDelayMs;
    private boolean powerTakeoverArmed;
    private double speedOffset;
    private double pitchOffset;
    private boolean automaticSetpointActive;

    public LauncherCommand(
            Shooter shooter,
            Feeder feeder,
            Gamepad gamepad,
            DistanceShotTaskHook distanceShotTaskHook) {
        this.shooter = shooter;
        this.feeder = feeder;
        this.gamepad = gamepad;
        this.distanceShotTaskHook = distanceShotTaskHook;
        addRequirements(shooter, feeder);
    }

    @Override
    public void initialize() {
        previousUp = false;
        previousDown = false;
        previousLeft = false;
        previousRight = false;
        previousY = false;
        previousB = false;
        previousRightTrigger = 0.0;
        rightTriggerPressedAtMs = 0;
        stableStartedAtMs = 0;
        fireStartedAtMs = 0;
        firing = false;
        stable = false;
        retracting = false;
        powerTakeoverArmed = false;
        speedOffset = 0.0;
        pitchOffset = 0.0;
        automaticSetpointActive = false;
        shooter.setPitchPosition(pitchPosition);
        shooter.idle();
        feeder.stop();
    }

    @Override
    public void execute() {
        distanceShotTaskHook.update();
        ShotLookupTable.Setpoint automaticSetpoint =
                distanceShotTaskHook.getSetpoint().orElse(null);
        automaticSetpointActive = automaticSetpoint != null;
        updateTuningButtons();
        if (automaticSetpointActive) {
            targetSpeed = Range.clip(
                    automaticSetpoint.targetSpeed + speedOffset,
                    0.0, ShooterConstants.speedMax);
            pitchPosition = Range.clip(
                    automaticSetpoint.pitchPosition + pitchOffset,
                    ShooterConstants.pitchMinPosition,
                    ShooterConstants.pitchMaxPosition);
        }
        shooter.setPitchPosition(pitchPosition);

        long now = System.currentTimeMillis();
        double leftTrigger = gamepad.left_trigger;
        double rightTrigger = gamepad.right_trigger;
        boolean reverseIntakeOnly = gamepad.left_bumper;
        boolean reverseIntakeAndTransit = gamepad.right_bumper;
        double currentSpeed = shooter.getMinimumVelocity();

        speedInTolerance = currentSpeed >= targetSpeed
                * (1.0 - ShooterConstants.speedToleranceFraction)
                && currentSpeed <= targetSpeed
                * (1.0 + ShooterConstants.speedToleranceFraction);

        if (rightTrigger > 0.1) {
            if (speedInTolerance) {
                if (stableStartedAtMs == 0) stableStartedAtMs = now;
                if (now - stableStartedAtMs >= ShooterConstants.stableTimeMs) stable = true;
            } else {
                stableStartedAtMs = 0;
                stable = false;
            }
        } else {
            stableStartedAtMs = 0;
            stable = false;
        }

        if (rightTrigger > 0.1 && stable && !firing) {
            firing = true;
            fireStartedAtMs = now;
        }
        if (rightTrigger <= 0.1
                && now - fireStartedAtMs > ShooterConstants.fireLockoutMs) {
            firing = false;
        }

        if (rightTrigger > 0.1) {
            powerTakeoverArmed = firing
                    && now - fireStartedAtMs >= powerTakeoverDelayMs;
            shooter.setVelocity(targetSpeed, powerTakeoverArmed);
        } else {
            powerTakeoverArmed = false;
            shooter.idle();
        }

        if (rightTrigger > 0.1 && previousRightTrigger <= 0.1) {
            rightTriggerPressedAtMs = now;
        }
        retracting = rightTrigger > 0.1
                && now - rightTriggerPressedAtMs < FeederConstants.retractTimeMs;

        double intakePower = 0.0;
        double transitPower = 0.0;
        boolean openLimit = false;
        if (reverseIntakeAndTransit) {
            intakePower = -FeederConstants.intakeSign;
            transitPower = -FeederConstants.transitIntakeSign;
        } else if (reverseIntakeOnly) {
            intakePower = -FeederConstants.intakeSign;
        } else if (retracting) {
            transitPower = -FeederConstants.transitFireSign;
        } else if (rightTrigger > 0.1 && (stable || firing)) {
            openLimit = true;
            intakePower = FeederConstants.intakeSign * rightTrigger
                    * FeederConstants.fireIntakePowerScale;
            transitPower = FeederConstants.transitFireSign * rightTrigger
                    * FeederConstants.fireTransitPowerScale;
        } else if (leftTrigger > 0.1) {
            intakePower = FeederConstants.intakeSign * leftTrigger;
            transitPower = FeederConstants.transitIntakeSign * leftTrigger;
        }
        feeder.setOutputs(intakePower, transitPower, openLimit);
        previousRightTrigger = rightTrigger;
    }

    @Override
    public void end(boolean interrupted) {
        stableStartedAtMs = 0;
        stable = false;
        firing = false;
        retracting = false;
        powerTakeoverArmed = false;
        shooter.stop();
        feeder.stop();
    }

    private void updateTuningButtons() {
        if (gamepad.dpad_left && !previousLeft) {
            if (automaticSetpointActive) {
                speedOffset -= ShooterConstants.speedStep;
            } else {
                targetSpeed = Range.clip(targetSpeed - ShooterConstants.speedStep,
                        0.0, ShooterConstants.speedMax);
            }
        }
        if (gamepad.dpad_right && !previousRight) {
            if (automaticSetpointActive) {
                speedOffset += ShooterConstants.speedStep;
            } else {
                targetSpeed = Range.clip(targetSpeed + ShooterConstants.speedStep,
                        0.0, ShooterConstants.speedMax);
            }
        }
        if (gamepad.dpad_up && !previousUp) {
            if (automaticSetpointActive) {
                pitchOffset += ShooterConstants.pitchStep;
            } else {
                pitchPosition = Range.clip(pitchPosition + ShooterConstants.pitchStep,
                        ShooterConstants.pitchMinPosition, ShooterConstants.pitchMaxPosition);
            }
        }
        if (gamepad.dpad_down && !previousDown) {
            if (automaticSetpointActive) {
                pitchOffset -= ShooterConstants.pitchStep;
            } else {
                pitchPosition = Range.clip(pitchPosition - ShooterConstants.pitchStep,
                        ShooterConstants.pitchMinPosition, ShooterConstants.pitchMaxPosition);
            }
        }
        if (gamepad.y && !previousY) {
            powerTakeoverDelayMs = Math.min(
                    ShooterConstants.powerTakeoverDelayMaxMs,
                    powerTakeoverDelayMs + ShooterConstants.powerTakeoverDelayStepMs);
        }
        if (gamepad.b && !previousB) {
            powerTakeoverDelayMs = Math.max(
                    ShooterConstants.powerTakeoverDelayMinMs,
                    powerTakeoverDelayMs - ShooterConstants.powerTakeoverDelayStepMs);
        }
        previousUp = gamepad.dpad_up;
        previousDown = gamepad.dpad_down;
        previousLeft = gamepad.dpad_left;
        previousRight = gamepad.dpad_right;
        previousY = gamepad.y;
        previousB = gamepad.b;
    }

    public double getTargetSpeed() {
        return targetSpeed;
    }

    public boolean isSpeedInTolerance() {
        return speedInTolerance;
    }

    public boolean isStable() {
        return stable;
    }

    public boolean isFiring() {
        return firing;
    }

    public boolean isRetracting() {
        return retracting;
    }

    public long getPowerTakeoverDelayMs() {
        return powerTakeoverDelayMs;
    }

    public boolean isPowerTakeoverArmed() {
        return powerTakeoverArmed;
    }

    public boolean isAutomaticSetpointActive() {
        return automaticSetpointActive;
    }

    public double getSpeedOffset() {
        return speedOffset;
    }

    public double getPitchOffset() {
        return pitchOffset;
    }

    public long getFiringElapsedMs() {
        return firing ? System.currentTimeMillis() - fireStartedAtMs : 0;
    }

    public long getStableElapsedMs() {
        return stableStartedAtMs == 0 ? 0
                : System.currentTimeMillis() - stableStartedAtMs;
    }
}
