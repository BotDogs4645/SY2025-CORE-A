package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

/**
 * IO interface for the Elevator subsystem.
 * This abstraction enables replay and simulation support through AdvantageKit.
 */
public interface ElevatorIO {

    @AutoLog
    class ElevatorIOInputs {
        public double positionRotations = 0.0;
        public double velocityRotationsPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double tempCelsius = 0.0;
        public boolean reverseLimitSwitch = false;
        public String controlMode = "Unknown";
    }

    /** update the set of loggable inputs */
    default void updateInputs(ElevatorIOInputs inputs) {}

    /** set the target position in rotations */
    default void setTargetPosition(double rotations) {}

    /** set duty cycle output (-1 to 1) */
    default void setDutyCycle(double dutyCycle) {}

    /** set brake mode (true) or coast mode (false) */
    default void setBrakeMode(boolean brake) {}

    /** reset the position to a given value */
    default void resetPosition(double rotations) {}
}

