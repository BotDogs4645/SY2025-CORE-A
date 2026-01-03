package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Climber subsystem with AdvantageKit replay support.
 * Uses the IO pattern to abstract hardware for logging and simulation.
 */
public class Climber extends SubsystemBase {
    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    private final Alert encoderAlert;

    public Climber(ClimberIO io) {
        this.io = io;
        encoderAlert = new Alert("Climber encoder disconnected", AlertType.kError);
    }

    @Override
    public void periodic() {
        // update and log inputs from hardware
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);

        // update alerts
        encoderAlert.set(!inputs.absoluteEncoderConnected);
    }

    public double getPosition() {
        return inputs.positionRotations;
    }

    public double getVelocity() {
        return inputs.velocityRotationsPerSec;
    }

    public void setDutyCycle(double dutyCycle) {
        io.setDutyCycle(dutyCycle);
    }

    public void setBrake() {
        io.setBrake();
    }
}

