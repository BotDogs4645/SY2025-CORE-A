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
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.commands.components.EndEffectorComponents;

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

    public double offset = 0.0;
    public double setpoint = 0.0;

    public boolean deployed = false;

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
                .withKG(EndEffectorConstants.KG)
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
        manipulateMotor.setControl(new StaticBrake());
    }

    public void setWheelDutyCycle(double speed) {
        manipulateMotor.set(speed);
    }
    public void setWheelBrake() {
        manipulateMotor.setControl(new StaticBrake());
    }

    public void setWheelCoast() {
        manipulateMotor.setControl(new CoastOut());
    }

    public void setPivotCoast() {
        pivotMotor.setControl(new CoastOut());
    }

    public String getWheelControl() {
        return manipulateMotor.getAppliedControl().getName();
    }

    public void setPivotPosition(Rotation2d position) {
        setpoint = position.getRotations();
        pivotControl.Position = setpoint + offset;
        pivotMotor.setControl(pivotControl);
    }

    public void updatePivotPosition() {
        pivotControl.Position = setpoint + offset;
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

    public boolean isDeployed() {
        return deployed;
    }

    public void increaseOffset() {
        offset -= 0.01;
        updatePivotPosition();
    }
    
    public void decreaseOffset() {
        offset += 0.01;
        updatePivotPosition();
    }

    public boolean hasReachedTarget() {
        return pivotMotor.getAppliedControl() == pivotControl
            && Math.abs(getPivotTargetPosition() - getPivotPosition()) <= EndEffectorConstants.rotationThreshold.getRotations()
            && Math.abs(getPivotVelocity()) <= EndEffectorConstants.velocityThreshold.in(RadiansPerSecond);
    }

    private boolean sensorTripped(LaserCan sensor, Alert alert, double threshold) {
        Measurement measurement = sensor.getMeasurement();
        if (measurement == null) {
            alert.set(true);
            return false;
        }
        alert.set(false);
        return measurement.distance_mm <= threshold;
    }

    public boolean firstCoralSensorTripped() { return sensorTripped(firstCoralSensor, firstCoralSensorAlert, EndEffectorConstants.coralThreshold.in(Millimeters)); }
    public boolean secondCoralSensorTripped() { return sensorTripped(secondCoralSensor, secondCoralSensorAlert, EndEffectorConstants.coralThreshold.in(Millimeters)); }


    public boolean algaeSensorTripped() {
        Measurement measurement = algaeSensor.getMeasurement();
        if (measurement == null) {
            algaeSensorAlert.set(true);
            return false;
        }
        algaeSensorAlert.set(false);
        Logger.recordOutput("EndEffector/algaeDistancemm", measurement.distance_mm);
        Logger.recordOutput("EndEffector/algaeAmbient", measurement.ambient);
        Logger.recordOutput("EndEffector/algaeStatus", measurement.status);
        return measurement.distance_mm <= EndEffectorConstants.algaeThreshold.in(Millimeters)
            && measurement.status == 0;
    }

    public boolean isSafeToElevate() {
        return getPivotPosition() >= EndEffectorConstants.safetyAngle.getRotations();
    }

    @Override
    public void periodic() {
        Logger.recordOutput("EndEffector/pivotPosition", getPivotPosition());
        Logger.recordOutput("EndEffector/encoderPosition", pivotEncoder.get());
        Logger.recordOutput("EndEffector/pivotSetpoint", getPivotTargetPosition());
        Logger.recordOutput("EndEffector/pivotVelocity", getPivotVelocity());
        Logger.recordOutput("EndEffector/voltageOut", pivotMotor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("EndEffector/pivotControl", pivotMotor.getAppliedControl().getName());
        Logger.recordOutput("EndEffector/pivotDone", hasReachedTarget());
        Logger.recordOutput("EndEffector/firstSensor", firstCoralSensorTripped());
        Logger.recordOutput("EndEffector/secondSensor", secondCoralSensorTripped());
        Logger.recordOutput("EndEffector/algaeSensor", algaeSensorTripped());
        Logger.recordOutput("EndEffector/wheelControl", getWheelControl());

        encoderAlert.set(!pivotEncoder.isConnected());
        if (secondCoralSensorTripped() && !firstCoralSensorTripped() && getWheelControl().equals("StaticBrake") && !DriverStation.isAutonomous()) {
            EndEffectorComponents.reverseCoral(this).schedule();
        }
    }
}