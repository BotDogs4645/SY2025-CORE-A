package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

/**
 * Elevator subsystem with AdvantageKit replay support.
 * Uses the IO pattern to abstract hardware for logging and simulation.
 */
public class Elevator extends SubsystemBase {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final Supplier<Boolean> safetySupplier;

    private double targetPositionRotations = 0.0;

    public Elevator(ElevatorIO io, Supplier<Boolean> safetySupplier) {
        this.io = io;
        this.safetySupplier = safetySupplier;
    }

    @Override
    public void periodic() {
        // update and log inputs from hardware
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        // log derived/calculated values as outputs
        Logger.recordOutput("Elevator/positionMeters", getPosition());
        Logger.recordOutput("Elevator/setpointMeters", getTargetPosition());
        Logger.recordOutput("Elevator/velocityMetersPerSec", getVelocity());
        Logger.recordOutput("Elevator/hasReachedTarget", hasReachedTarget());
        Logger.recordOutput("Elevator/safeToMove", safetySupplier.get());
    }

    /**
     * Get the current elevator position in meters.
     */
    public double getPosition() {
        return inputs.positionRotations * ElevatorConstants.metersPerRotation;
    }

    /**
     * Get the current elevator velocity in meters per second.
     */
    public double getVelocity() {
        return inputs.velocityRotationsPerSec * ElevatorConstants.metersPerRotation;
    }

    /**
     * Get the target position in meters.
     */
    public double getTargetPosition() {
        return targetPositionRotations * ElevatorConstants.metersPerRotation;
    }

    /**
     * Check if the elevator has reached its target position.
     */
    public boolean hasReachedTarget() {
        return inputs.controlMode.equals("PositionDutyCycle")
            && Math.abs(getTargetPosition() - getPosition()) <= ElevatorConstants.positionThreshold.in(Meters)
            && Math.abs(getVelocity()) <= ElevatorConstants.velocityThreshold.in(MetersPerSecond);
    }

    /**
     * Set the brake mode for the elevator.
     * @param brake true for brake mode, false for coast mode
     */
    public void setBrake(boolean brake) {
        io.setBrakeMode(brake);
    }

    /**
     * Set the target position for the elevator.
     * @param distance the target distance
     */
    public void setTarget(Distance distance) {
        if (!safetySupplier.get()) {
            return;
        }
        targetPositionRotations = distance.in(Meters) / ElevatorConstants.metersPerRotation;
        io.setTargetPosition(targetPositionRotations);
    }

    /**
     * Set the duty cycle for manual control.
     * @param dutyCycle the duty cycle (-1 to 1)
     */
    public void setDutyCycle(double dutyCycle) {
        if (!safetySupplier.get()) {
            return;
        }
        io.setDutyCycle(dutyCycle);
    }

    /**
     * Check if the reverse limit switch is triggered.
     */
    public boolean isAtBottomLimit() {
        return inputs.reverseLimitSwitch;
    }
}

