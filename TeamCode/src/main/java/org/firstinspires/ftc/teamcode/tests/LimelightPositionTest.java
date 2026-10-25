package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.vision.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;

/** Vision-only test: requires only the configured Limelight. */
@TeleOp(name = "Test: Limelight Position Only", group = "Test")
public class LimelightPositionTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Vision vision = new Vision(hardwareMap, Alliance.BLUE);
        boolean previousX = false;
        try {
            waitForStart();
            while (opModeIsActive()) {
                if (gamepad1.x && !previousX) {
                    vision.setAlliance(vision.getAlliance() == Alliance.BLUE ? Alliance.RED : Alliance.BLUE);
                }
                previousX = gamepad1.x;
                vision.update();
                telemetry.addData("Alliance", vision.getAlliance());
                telemetry.addData("Pipeline expected/actual", "%d / %d",
                        vision.getExpectedPipelineIndex(), vision.getActualPipelineIndex());
                telemetry.addData("Status", vision.getPoseStatus());
                telemetry.addData("Tags", vision.getVisibleTagIds());
                Pose2D pose = vision.getCorrectedRobotFieldPose().orElse(null);
                telemetry.addData("Position valid", pose != null);
                if (pose != null) {
                    telemetry.addData("Field X/Y (cm), heading (deg)", "%.2f / %.2f / %.2f",
                            pose.getX(DistanceUnit.CM), pose.getY(DistanceUnit.CM),
                            pose.getHeading(AngleUnit.DEGREES));
                }
                telemetry.addData("Hive angle (deg)", vision.getHiveAngleDegrees());
                telemetry.addLine("Origin: field center. +X: audience. +Y: blue HIVE.");
                telemetry.addLine("X: switch blue/red pipeline");
                telemetry.update();
                idle();
            }
        } finally {
            vision.stop();
        }
    }
}
