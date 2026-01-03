package frc.robot.subsystems.chute;

import org.littletonrobotics.junction.AutoLog;

/**
 * IO interface for the Chute subsystem.
 * This abstraction enables replay and simulation support through AdvantageKit.
 */
public interface ChuteIO {

    @AutoLog
    class ChuteIOInputs {
        public double positionRotations = 0.0;
        public double velocityRotationsPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public String controlMode = "Unknown";

        // absolute encoder
        public double absoluteEncoderPositionRotations = 0.0;
        public boolean absoluteEncoderConnected = false;
    }

    /** update the set of loggable inputs */
    default void updateInputs(ChuteIOInputs inputs) {}

    /** set the target position in rotations */
    default void setTargetPosition(double rotations) {}

    /** set to coast mode */
    default void setCoast() {}

    /** sync motor position from absolute encoder */
    default void syncFromAbsoluteEncoder() {}
}

