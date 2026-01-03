package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * IO interface for Vision subsystem cameras.
 * This abstraction enables replay and simulation support through AdvantageKit.
 */
public interface VisionIO {

    @AutoLog
    class VisionIOInputs {
        public boolean connected = false;
        public double latestTimestamp = 0.0;
        public Rotation2d latestTx = new Rotation2d();
        public Rotation2d latestTy = new Rotation2d();
        public int[] tagIds = new int[0];
        public PoseObservation[] poseObservations = new PoseObservation[0];
    }

    /** Represents a robot pose observation from vision */
    record PoseObservation(
        double timestamp,
        Pose3d pose,
        double ambiguity,
        int tagCount,
        double averageTagDistance,
        PoseObservationType type
    ) {}

    enum PoseObservationType {
        MEGATAG_1,
        MEGATAG_2
    }

    /** update the set of loggable inputs */
    default void updateInputs(VisionIOInputs inputs, double robotHeadingDegrees) {}
}

