package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.EnumMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

// Localization extracted from FTC-16095-FatDragon-2027-Public; no drive/aim/shoot logic.
public class Vision extends SubsystemBase {
    private final Limelight3A limelight;
    private Alliance alliance;
    private int pipelineIndex;
    private String poseStatus = "Waiting for AprilTags";
    private Pose3D rawRobotFieldPose;
    private Pose2D correctedRobotFieldPose;
    private double frameTimestamp = Double.NaN;

    public double getFrameTimestamp() {
        return frameTimestamp;
    }
    private int botposeTagCount;
    private int usableTagCount;
    private int actualPipelineIndex = -1;
    private long resultAgeMs = -1;
    private double lastPhysicalFrameTimestamp = Double.NaN;
    private long lastPhysicalFrameSeenNanos;
    private long physicalFrameAgeMs = -1;
    private boolean rawResultValid;
    private boolean hiveSolveSuccessful;
    private VisionMath.HiveSolveResult hiveSolveResult;
    private double averageTagDistanceMeters = Double.NaN;
    private double tagSpanMeters = Double.NaN;
    private double averageTagArea = Double.NaN;
    private double[] mt1StandardDeviations;
    private double hiveAngleDegrees = Double.NaN;
    private double robotLevelErrorDegrees = Double.NaN;
    private List<Integer> visibleTagIds = Collections.emptyList();
    private Map<Integer, Pose3D> visibleTagPosesRobotSpace = Collections.emptyMap();

    public Vision(HardwareMap hardwareMap, Alliance alliance) {
        this.alliance = alliance;
        pipelineIndex = alliance == Alliance.BLUE
                ? VisionConstants.bluePipelineIndex : VisionConstants.redPipelineIndex;
        limelight = hardwareMap.get(Limelight3A.class, VisionConstants.limelightName);
        limelight.setPollRateHz(VisionConstants.pollRateHz);
        limelight.pipelineSwitch(pipelineIndex);
        limelight.start();
    }

    @Override
    public void periodic() {
        update();
    }

    public void update() {
        clearCurrentFrame();
        if (!limelight.isConnected()) {
            poseStatus = "Limelight disconnected";
            return;
        }
        LLResult result = limelight.getLatestResult();
        if (result == null) {
            poseStatus = "No Limelight result";
            return;
        }
        actualPipelineIndex = result.getPipelineIndex();
        resultAgeMs = result.getStaleness();
        rawResultValid = result.isValid();
        if (!rawResultValid) {
            poseStatus = "Limelight result invalid";
            return;
        }
        if (actualPipelineIndex != pipelineIndex) {
            poseStatus = "Waiting for pipeline " + pipelineIndex + "; current " + actualPipelineIndex;
            return;
        }
        if (resultAgeMs < 0 || resultAgeMs > VisionConstants.maxResultAgeMs) {
            poseStatus = "Limelight result stale: " + resultAgeMs + " ms";
            return;
        }
        frameTimestamp = result.getTimestamp();
        if (!Double.isFinite(frameTimestamp) || frameTimestamp <= 0.0) {
            poseStatus = "Invalid Limelight frame timestamp";
            return;
        }
        long nowNanos = System.nanoTime();
        if (frameTimestamp != lastPhysicalFrameTimestamp) {
            lastPhysicalFrameTimestamp = frameTimestamp;
            lastPhysicalFrameSeenNanos = nowNanos;
        }
        // SDK staleness measures HTTP reception age, not whether the camera frame advanced.
        physicalFrameAgeMs = (nowNanos - lastPhysicalFrameSeenNanos) / 1_000_000L;
        if (physicalFrameAgeMs > VisionConstants.maxResultAgeMs) {
            poseStatus = "Limelight physical frame frozen: " + physicalFrameAgeMs + " ms";
            return;
        }
        averageTagDistanceMeters = result.getBotposeAvgDist();
        tagSpanMeters = result.getBotposeSpan();
        averageTagArea = result.getBotposeAvgArea();
        double[] stddev = result.getStddevMt1();
        mt1StandardDeviations = stddev == null ? null : stddev.clone();
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {
            poseStatus = "No AprilTag detections";
            return;
        }
        List<Integer> ids = new ArrayList<>();
        Map<Integer, Pose3D> poses = new LinkedHashMap<>();
        boolean foreignAllianceTag = false;
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int id = fiducial.getFiducialId();
            Optional<CellCluster> cluster = CellCluster.forTagId(id);
            if (!cluster.isPresent()) {
                continue;
            }
            if (!cluster.get().isForAlliance(alliance)) {
                foreignAllianceTag = true;
                continue;
            }
            ids.add(id);
            Pose3D pose = fiducial.getTargetPoseRobotSpace();
            if (VisionMath.getRobotSpacePositionInches(pose).isPresent()) {
                poses.put(id, pose);
            }
            // JSON fiducial ta is a fraction; compare areas in percentage units.
            double area = fiducial.getTargetArea() * 100.0;
            if (Double.isFinite(area) && area >= VisionConstants.minimumTargetArea) {
                usableTagCount++;
            }
        }
        visibleTagIds = Collections.unmodifiableList(ids);
        visibleTagPosesRobotSpace = Collections.unmodifiableMap(poses);
        if (foreignAllianceTag) {
            poseStatus = "Rejected mixed-alliance HIVE tags";
            return;
        }
        botposeTagCount = result.getBotposeTagCount();
        rawRobotFieldPose = result.getBotpose();
        if (botposeTagCount < VisionConstants.minimumBotposeTagCount) {
            poseStatus = "Need at least " + VisionConstants.minimumBotposeTagCount
                    + " MT1 localization tags; got " + botposeTagCount;
            return;
        }
        if (usableTagCount < VisionConstants.minimumBotposeTagCount) {
            poseStatus = "Need at least " + VisionConstants.minimumBotposeTagCount
                    + " alliance tags with area >= " + VisionConstants.minimumTargetArea
                    + "%; got " + usableTagCount;
            return;
        }
        hiveSolveResult = VisionMath.solveHiveFlatPose(
                rawRobotFieldPose, VisionConstants.hivePivotHeightInches,
                VisionConstants.maximumHiveTiltDegrees, VisionConstants.robotLevelToleranceDegrees
        );
        poseStatus = hiveSolveResult.status;
        hiveSolveSuccessful = hiveSolveResult.correction != null;
        if (!hiveSolveSuccessful) {
            return;
        }
        correctedRobotFieldPose = hiveSolveResult.correction.robotPose;
        hiveAngleDegrees = hiveSolveResult.correction.hiveAngleDegrees;
        robotLevelErrorDegrees = hiveSolveResult.correction.levelErrorDegrees;
        poseStatus = "Corrected MT1 field pose ready";
    }

    /** Only pipeline selection is adapted for the existing X-button test. */
    public void setAlliance(Alliance selected) {
        if (selected == null) throw new IllegalArgumentException("Alliance required");
        alliance = selected;
        pipelineIndex = selected == Alliance.BLUE
                ? VisionConstants.bluePipelineIndex : VisionConstants.redPipelineIndex;
        limelight.pipelineSwitch(pipelineIndex);
        lastPhysicalFrameTimestamp = Double.NaN;
        clearCurrentFrame();
    }
    public Alliance getAlliance() { return alliance; }
    public int getExpectedPipelineIndex() { return pipelineIndex; }
    public int getActualPipelineIndex() { return actualPipelineIndex; }
    public String getPoseStatus() { return poseStatus; }
    public List<Integer> getVisibleTagIds() { return visibleTagIds; }
    public double getHiveAngleDegrees() { return hiveAngleDegrees; }
    public double getRobotLevelErrorDegrees() { return robotLevelErrorDegrees; }
    public Optional<Pose3D> getRawRobotFieldPose() { return Optional.ofNullable(rawRobotFieldPose); }
    public Optional<Pose2D> getCorrectedRobotFieldPose() { return Optional.ofNullable(correctedRobotFieldPose); }
    public void stop() { limelight.stop(); clearCurrentFrame(); }

    private void clearCurrentFrame() {
        poseStatus = "Waiting for AprilTags";
        rawRobotFieldPose = null;
        correctedRobotFieldPose = null;
        frameTimestamp = Double.NaN;
        botposeTagCount = 0;
        usableTagCount = 0;
        actualPipelineIndex = -1;
        resultAgeMs = -1;
        physicalFrameAgeMs = -1;
        rawResultValid = false;
        hiveSolveSuccessful = false;
        hiveSolveResult = null;
        averageTagDistanceMeters = Double.NaN;
        tagSpanMeters = Double.NaN;
        averageTagArea = Double.NaN;
        mt1StandardDeviations = null;
        hiveAngleDegrees = Double.NaN;
        robotLevelErrorDegrees = Double.NaN;
        visibleTagIds = Collections.emptyList();
        visibleTagPosesRobotSpace = Collections.emptyMap();
    }
}
