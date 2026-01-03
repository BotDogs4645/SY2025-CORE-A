package frc.robot.subsystems.chute;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ChuteConstants;

/**
 * Chute subsystem with AdvantageKit replay support.
 * Uses the IO pattern to abstract hardware for logging and simulation.
 */
public class Chute extends SubsystemBase {
    private final ChuteIO io;
    private final ChuteIOInputsAutoLogged inputs = new ChuteIOInputsAutoLogged();

    private final Alert encoderAlert;

    private double setpoint = 0.0;

    public Chute(ChuteIO io) {
        this.io = io;
        encoderAlert = new Alert("Chute encoder disconnected", AlertType.kError);
    }

    @Override
    public void periodic() {
        // update and log inputs from hardware
        io.updateInputs(inputs);
        Logger.processInputs("Chute", inputs);

        // log derived values as outputs
        Logger.recordOutput("Chute/setpoint", setpoint);
        Logger.recordOutput("Chute/hasReachedTarget", hasReachedTarget());

        // update alerts
        encoderAlert.set(!inputs.absoluteEncoderConnected);
    }

    public void setPosition(Rotation2d position) {
        setpoint = position.getRotations();
        io.setTargetPosition(setpoint);
    }

    public double getPosition() {
        return inputs.positionRotations;
    }

    public double getVelocity() {
        return inputs.velocityRotationsPerSec;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public boolean hasReachedTarget() {
        return inputs.controlMode.equals("PositionVoltage")
            && Math.abs(getPosition() - getSetpoint()) <= ChuteConstants.rotationThreshold.getRotations()
            && Math.abs(getVelocity()) <= ChuteConstants.velocityThreshold.in(RadiansPerSecond);
    }
}

