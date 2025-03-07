package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;

public class EndEffector extends SubsystemBase {
    private final TalonFX manipulateMotor;
    private final TalonFX pivotMotor;
    private final DutyCycleEncoder pivotEncoder;

    private final LaserCan firstCoralSensor;
    private final LaserCan secondCoralSensor;
    private final LaserCan algaeSensor;

    private final MotionMagicVoltage pivotControl;

    private final Alert firstCoralSensorAlert;
    private final Alert secondCoralSensorAlert;
    private final Alert algaeSensorAlert;
    private final Alert encoderAlert;

    public EndEffector() {
        firstCoralSensor = new LaserCan(EndEffectorConstants.firstCoralSensorID);
        secondCoralSensor = new LaserCan(EndEffectorConstants.secondCoralSensorID);
        algaeSensor = new LaserCan(EndEffectorConstants.algaeSensorID);

        firstCoralSensorAlert = new Alert("First Coral LaserCAN failed to read", AlertType.kWarning);
        secondCoralSensorAlert = new Alert("Second Coral LaserCAN failed to read", AlertType.kWarning);
        algaeSensorAlert = new Alert("Algae LaserCAN failed to read", AlertType.kWarning);
        encoderAlert = new Alert("EndEffector pivot encoder disconnected", AlertType.kError);

        manipulateMotor = new TalonFX(EndEffectorConstants.manipulateMotorID);
        pivotMotor = new TalonFX(EndEffectorConstants.pivotMotorID);
        pivotEncoder = new DutyCycleEncoder(EndEffectorConstants.encoderDioPort);

        var pivotConfig = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake)
            ).withFeedback(new FeedbackConfigs()
                .withSensorToMechanismRatio(EndEffectorConstants.gearRatio)
            ).withSlot0(new Slot0Configs()
                //.withKS(0)
                //.withKV(0)
                //.withKA(0)
                .withKP(EndEffectorConstants.KP)
                .withKI(EndEffectorConstants.KI)
                .withKD(EndEffectorConstants.KD)
            ).withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(80 / EndEffectorConstants.gearRatio)
                .withMotionMagicAcceleration(160 / EndEffectorConstants.gearRatio)
                .withMotionMagicJerk(1600 / EndEffectorConstants.gearRatio)
            );

        pivotMotor.getConfigurator().apply(pivotConfig);

        manipulateMotor.getConfigurator().apply(new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive))
        );
        
        pivotControl = new MotionMagicVoltage(0);
        pivotMotor.setPosition(pivotEncoder.get());

        pivotMotor.setControl(new CoastOut());
    }

    public void setWheelDutyCycle(double speed) {
        manipulateMotor.set(speed);
    }

    public void setPivotPosition(Rotation2d position) {
        pivotControl.Position = position.getRotations();
        pivotMotor.setControl(pivotControl);
    }

    public double getPivotPosition() {
        return pivotMotor.getPosition().getValueAsDouble();
    }

    public double getPivotVelocity() {
        return pivotMotor.getVelocity().getValueAsDouble();
    }

    public double getPivotTargetPosition() {
        return pivotControl.Position;
    }

    public boolean hasReachedTarget() {
        return pivotMotor.getAppliedControl() == pivotControl
            && Math.abs(getPivotTargetPosition() - getPivotPosition()) <= EndEffectorConstants.rotationThreshold.getRotations()
            && Math.abs(getPivotVelocity()) <= EndEffectorConstants.velocityThreshold.in(RadiansPerSecond);
    }

    public boolean firstCoralSensorTripped() {
        Measurement measurement = firstCoralSensor.getMeasurement();
        if (measurement == null) {
            firstCoralSensorAlert.set(true);
            return false;
        }
        firstCoralSensorAlert.set(false);
        return measurement.distance_mm <= EndEffectorConstants.coralThreshold.in(Millimeters);
    }

    public boolean secondCoralSensorTripped() {
        Measurement measurement = secondCoralSensor.getMeasurement();
        if (measurement == null) {
            secondCoralSensorAlert.set(true);
            return false;
        }
        secondCoralSensorAlert.set(false);
        return measurement.distance_mm <= EndEffectorConstants.coralThreshold.in(Millimeters);
    }

    public boolean algaeSensorTripped() {
        Measurement measurement = algaeSensor.getMeasurement();
        if (measurement == null) {
            algaeSensorAlert.set(true);
            return false;
        }
        algaeSensorAlert.set(false);
        return measurement.distance_mm <= EndEffectorConstants.algaeThreshold.in(Millimeters);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("EndEffector/pivotPosition", getPivotPosition());
        Logger.recordOutput("EndEffector/pivotSetpoint", getPivotTargetPosition());
        Logger.recordOutput("EndEffector/pivotVelocity", getPivotVelocity());
        Logger.recordOutput("EndEffector/voltageOut", pivotMotor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("EndEffector/pivotControl", pivotMotor.getAppliedControl().getName());
        Logger.recordOutput("EndEffector/pivotDone", hasReachedTarget());
        Logger.recordOutput("EndEffector/encoderPosition", pivotEncoder.get());
        Logger.recordOutput("EndEffector/firstSensor", firstCoralSensorTripped());
        Logger.recordOutput("EndEffector/secondSensor", secondCoralSensorTripped());
        Logger.recordOutput("EndEffector/algaeSensor", algaeSensorTripped());

        if (getPivotVelocity() < EndEffectorConstants.rotationThreshold.getRotations() && pivotEncoder.isConnected()) {
            pivotMotor.setPosition(pivotEncoder.get(), 0); // 0 second timeout (do not wait for status)
        }

        encoderAlert.set(!pivotEncoder.isConnected());
    }
}