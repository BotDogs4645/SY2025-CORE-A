package frc.robot.subsystems.endeffector;

import org.littletonrobotics.junction.AutoLog;

/**
 * IO interface for the EndEffector subsystem.
 * This abstraction enables replay and simulation support through AdvantageKit.
 */
public interface EndEffectorIO {

    @AutoLog
    class EndEffectorIOInputs {
        // pivot motor inputs
        public double pivotPositionRotations = 0.0;
        public double pivotVelocityRotationsPerSec = 0.0;
        public double pivotAppliedVolts = 0.0;
        public double pivotCurrentAmps = 0.0;
        public String pivotControlMode = "Unknown";

        // absolute encoder
        public double absoluteEncoderPositionRotations = 0.0;
        public boolean absoluteEncoderConnected = false;

        // manipulator motor inputs
        public double manipulatorVelocityRotationsPerSec = 0.0;
        public double manipulatorAppliedVolts = 0.0;
        public double manipulatorCurrentAmps = 0.0;
        public String manipulatorControlMode = "Unknown";

        // LaserCAN sensors
        public double firstCoralDistanceMm = 0.0;
        public boolean firstCoralSensorConnected = false;
        public double secondCoralDistanceMm = 0.0;
        public boolean secondCoralSensorConnected = false;
        public double algaeDistanceMm = 0.0;
        public int algaeStatus = 0;
        public int algaeAmbient = 0;
        public boolean algaeSensorConnected = false;
    }

    /** update the set of loggable inputs */
    default void updateInputs(EndEffectorIOInputs inputs) {}

    /** set the pivot target position in rotations */
    default void setPivotPosition(double rotations) {}

    /** set the pivot motor to coast mode */
    default void setPivotCoast() {}

    /** set the manipulator wheel duty cycle (-1 to 1) */
    default void setManipulatorDutyCycle(double dutyCycle) {}

    /** set the manipulator to brake mode */
    default void setManipulatorBrake() {}

    /** set the manipulator to coast mode */
    default void setManipulatorCoast() {}

    /** sync the pivot motor position from the absolute encoder */
    default void syncPivotFromAbsoluteEncoder() {}
}

