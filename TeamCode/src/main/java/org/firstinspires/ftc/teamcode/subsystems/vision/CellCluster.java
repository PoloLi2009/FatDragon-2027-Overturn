package org.firstinspires.ftc.teamcode.subsystems.vision;

import java.util.Optional;

public enum CellCluster {
    RED_OPPOSITE_AUDIENCE(Alliance.RED, 30, 33, 0),
    RED_AUDIENCE(Alliance.RED, 34, 37, 1),
    BLUE_AUDIENCE(Alliance.BLUE, 38, 41, 2),
    BLUE_OPPOSITE_AUDIENCE(Alliance.BLUE, 42, 45, 3);

    private final Alliance alliance;
    private final int firstTagId;
    private final int lastTagId;
    private final int tuningIndex;

    CellCluster(Alliance alliance, int firstTagId, int lastTagId, int tuningIndex) {
        this.alliance = alliance;
        this.firstTagId = firstTagId;
        this.lastTagId = lastTagId;
        this.tuningIndex = tuningIndex;
    }

    public boolean isForAlliance(Alliance requestedAlliance) {
        return alliance == requestedAlliance;
    }

    public boolean contains(int tagId) {
        return tagId >= firstTagId && tagId <= lastTagId;
    }

    public int getTuningIndex() {
        return tuningIndex;
    }

    /** Same alliance/half-field selection and nominal targets as the public project. */
    public static Optional<CellCluster> forFieldX(Alliance alliance, double fieldX) {
        if (alliance == null || !Double.isFinite(fieldX)) return Optional.empty();
        if (alliance == Alliance.RED) {
            return Optional.of(fieldX >= 0 ? RED_AUDIENCE : RED_OPPOSITE_AUDIENCE);
        }
        return Optional.of(fieldX >= 0 ? BLUE_AUDIENCE : BLUE_OPPOSITE_AUDIENCE);
    }

    public double getTargetXmm() {
        return (this == RED_AUDIENCE || this == BLUE_AUDIENCE ? 17.7 : -17.7) * 25.4;
    }

    public double getTargetYmm() {
        return (alliance == Alliance.BLUE ? 12.75 : -12.75) * 25.4;
    }

    /** CCW error for the robot REAR, not its front, to face this target. */
    public double rearHeadingErrorRadians(double xMm, double yMm, double headingRadians) {
        double dx = getTargetXmm() - xMm;
        double dy = getTargetYmm() - yMm;
        if (!Double.isFinite(dx) || !Double.isFinite(dy)
                || !Double.isFinite(headingRadians) || Math.hypot(dx, dy) < 1.0) {
            return Double.NaN;
        }
        return Math.IEEEremainder(Math.atan2(dy, dx) - Math.PI - headingRadians,
                2.0 * Math.PI);
    }

    public static Optional<CellCluster> forTagId(int tagId) {
        for (CellCluster cluster : values()) {
            if (cluster.contains(tagId)) {
                return Optional.of(cluster);
            }
        }
        return Optional.empty();
    }
}
