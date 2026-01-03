package frc.robot.subsystems.endeffector;

import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.commands.components.EndEffectorComponents;

/**
 * EndEffector subsystem with AdvantageKit replay support.
 * Uses the IO pattern to abstract hardware for logging and simulation.
 */
public class EndEffector extends SubsystemBase {
    private final EndEffectorIO io;
    private final EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();

    private final Alert firstCoralSensorAlert;
    private final Alert secondCoralSensorAlert;
    private final Alert algaeSensorAlert;
    private final Alert encoderAlert;

    private double setpoint = 0.0;
    public boolean deployed = false;

    public EndEffector(EndEffectorIO io) {
        this.io = io;

        firstCoralSensorAlert = new Alert("First Coral LaserCAN failed to read", AlertType.kWarning);
        secondCoralSensorAlert = new Alert("Second Coral LaserCAN failed to read", AlertType.kWarning);
        algaeSensorAlert = new Alert("Algae LaserCAN failed to read", AlertType.kWarning);
        encoderAlert = new Alert("EndEffector pivot encoder disconnected", AlertType.kError);
    }

    @Override
    public void periodic() {
        // update and log inputs from hardware
        io.updateInputs(inputs);
        Logger.processInputs("EndEffector", inputs);

        // log derived values as outputs
        Logger.recordOutput("EndEffector/pivotSetpoint", setpoint);
        Logger.recordOutput("EndEffector/hasReachedTarget", hasReachedTarget());
        Logger.recordOutput("EndEffector/firstSensorTripped", firstCoralSensorTripped());
        Logger.recordOutput("EndEffector/secondSensorTripped", secondCoralSensorTripped());
        Logger.recordOutput("EndEffector/algaeSensorTripped", algaeSensorTripped());
        Logger.recordOutput("EndEffector/isSafeToElevate", isSafeToElevate());

        // update alerts
        firstCoralSensorAlert.set(!inputs.firstCoralSensorConnected);
        secondCoralSensorAlert.set(!inputs.secondCoralSensorConnected);
        algaeSensorAlert.set(!inputs.algaeSensorConnected);
        encoderAlert.set(!inputs.absoluteEncoderConnected);

        // autocorrect coral position if needed
        if (secondCoralSensorTripped() && !firstCoralSensorTripped() 
            && inputs.manipulatorControlMode.equals("StaticBrake") 
            && !DriverStation.isAutonomous()) {
            EndEffectorComponents.reverseCoral(this).schedule();
        }
    }

    public void setWheelDutyCycle(double speed) {
        Logger.recordOutput("EndEffector/wheelSpeedCommand", speed);
        io.setManipulatorDutyCycle(speed);
    }

    public void setWheelBrake() {
        io.setManipulatorBrake();
    }

    public void setWheelCoast() {
        io.setManipulatorCoast();
    }

    public void setPivotCoast() {
        io.setPivotCoast();
    }

    public String getWheelControl() {
        return inputs.manipulatorControlMode;
    }

    public void setPivotPosition(Rotation2d position) {
        setpoint = position.getRotations();
        io.setPivotPosition(setpoint);
    }

    public double getPivotPosition() {
        return inputs.pivotPositionRotations;
    }

    public double getPivotVelocity() {
        return inputs.pivotVelocityRotationsPerSec;
    }

    public double getPivotTargetPosition() {
        return setpoint;
    }

    @AutoLogOutput
    public boolean isDeployed() {
        return deployed;
    }

    public boolean hasReachedTarget() {
        return inputs.pivotControlMode.equals("MotionMagicVoltage")
            && Math.abs(getPivotTargetPosition() - getPivotPosition()) <= EndEffectorConstants.rotationThreshold.getRotations()
            && Math.abs(getPivotVelocity()) <= EndEffectorConstants.velocityThreshold.in(RadiansPerSecond);
    }

    public boolean firstCoralSensorTripped() {
        if (!inputs.firstCoralSensorConnected) {
            return false;
        }
        return inputs.firstCoralDistanceMm <= EndEffectorConstants.coralThreshold.in(Millimeters);
    }

    public boolean secondCoralSensorTripped() {
        if (!inputs.secondCoralSensorConnected) {
            return false;
        }
        return inputs.secondCoralDistanceMm <= EndEffectorConstants.coralThreshold.in(Millimeters);
    }

    public boolean algaeSensorTripped() {
        if (!inputs.algaeSensorConnected) {
            return false;
        }
        return inputs.algaeDistanceMm <= EndEffectorConstants.algaeThreshold.in(Millimeters)
            && inputs.algaeStatus == 0;
    }

    public boolean isSafeToElevate() {
        return getPivotPosition() >= EndEffectorConstants.safetyAngle.getRotations();
    }
}

