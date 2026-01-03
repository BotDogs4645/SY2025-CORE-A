package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

/**
 * IO interface for the Climber subsystem.
 * This abstraction enables replay and simulation support through AdvantageKit.
 */
public interface ClimberIO {

    @AutoLog
    class ClimberIOInputs {
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
    default void updateInputs(ClimberIOInputs inputs) {}

    /** set duty cycle output (-1 to 1) */
    default void setDutyCycle(double dutyCycle) {}

    /** set brake mode */
    default void setBrake() {}

    /** sync motor position from absolute encoder */
    default void syncFromAbsoluteEncoder() {}
}

