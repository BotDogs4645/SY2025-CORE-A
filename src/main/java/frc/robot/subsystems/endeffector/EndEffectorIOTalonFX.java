package frc.robot.subsystems.endeffector;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.EndEffectorConstants;

/**
 * Real hardware implementation of EndEffectorIO using TalonFX motors and LaserCAN sensors.
 */
public class EndEffectorIOTalonFX implements EndEffectorIO {
    private final TalonFX manipulateMotor;
    private final TalonFX pivotMotor;
    private final DutyCycleEncoder pivotEncoder;

    private final LaserCan firstCoralSensor;
    private final LaserCan secondCoralSensor;
    private final LaserCan algaeSensor;

    private final MotionMagicVoltage pivotControl;

    // status signals for pivot motor
    private final StatusSignal<Angle> pivotPositionSignal;
    private final StatusSignal<AngularVelocity> pivotVelocitySignal;
    private final StatusSignal<Voltage> pivotVoltageSignal;
    private final StatusSignal<Current> pivotCurrentSignal;

    // status signals for manipulator motor
    private final StatusSignal<AngularVelocity> manipulatorVelocitySignal;
    private final StatusSignal<Voltage> manipulatorVoltageSignal;
    private final StatusSignal<Current> manipulatorCurrentSignal;

    private double pivotOffset = 0.0;

    public EndEffectorIOTalonFX() {
        // initialize sensors
        firstCoralSensor = new LaserCan(EndEffectorConstants.firstCoralSensorID);
        secondCoralSensor = new LaserCan(EndEffectorConstants.secondCoralSensorID);
        algaeSensor = new LaserCan(EndEffectorConstants.algaeSensorID);

        // initialize motors
        manipulateMotor = new TalonFX(EndEffectorConstants.manipulateMotorID);
        pivotMotor = new TalonFX(EndEffectorConstants.pivotMotorID);
        pivotEncoder = new DutyCycleEncoder(EndEffectorConstants.encoderDioPort);

        // configure pivot motor
        var pivotConfig = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake)
            ).withFeedback(new FeedbackConfigs()
                .withSensorToMechanismRatio(EndEffectorConstants.gearRatio)
            ).withSlot0(new Slot0Configs()
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

        // configure manipulator motor
        manipulateMotor.getConfigurator().apply(new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive))
        );

        // initialize control and sync position
        pivotControl = new MotionMagicVoltage(0);
        pivotMotor.setPosition(pivotEncoder.get());
        pivotMotor.setControl(new CoastOut());
        manipulateMotor.setControl(new StaticBrake());

        // cache status signals
        pivotPositionSignal = pivotMotor.getPosition();
        pivotVelocitySignal = pivotMotor.getVelocity();
        pivotVoltageSignal = pivotMotor.getMotorVoltage();
        pivotCurrentSignal = pivotMotor.getStatorCurrent();

        manipulatorVelocitySignal = manipulateMotor.getVelocity();
        manipulatorVoltageSignal = manipulateMotor.getMotorVoltage();
        manipulatorCurrentSignal = manipulateMotor.getStatorCurrent();

        // optimize CAN bus usage
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            pivotPositionSignal,
            pivotVelocitySignal,
            pivotVoltageSignal,
            pivotCurrentSignal,
            manipulatorVelocitySignal,
            manipulatorVoltageSignal,
            manipulatorCurrentSignal
        );
        pivotMotor.optimizeBusUtilization();
        manipulateMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(EndEffectorIOInputs inputs) {
        // refresh all motor signals
        BaseStatusSignal.refreshAll(
            pivotPositionSignal,
            pivotVelocitySignal,
            pivotVoltageSignal,
            pivotCurrentSignal,
            manipulatorVelocitySignal,
            manipulatorVoltageSignal,
            manipulatorCurrentSignal
        );

        // pivot motor inputs
        inputs.pivotPositionRotations = pivotPositionSignal.getValueAsDouble();
        inputs.pivotVelocityRotationsPerSec = pivotVelocitySignal.getValueAsDouble();
        inputs.pivotAppliedVolts = pivotVoltageSignal.getValueAsDouble();
        inputs.pivotCurrentAmps = pivotCurrentSignal.getValueAsDouble();
        inputs.pivotControlMode = pivotMotor.getAppliedControl().getName();

        // absolute encoder
        inputs.absoluteEncoderPositionRotations = pivotEncoder.get();
        inputs.absoluteEncoderConnected = pivotEncoder.isConnected();

        // manipulator motor inputs
        inputs.manipulatorVelocityRotationsPerSec = manipulatorVelocitySignal.getValueAsDouble();
        inputs.manipulatorAppliedVolts = manipulatorVoltageSignal.getValueAsDouble();
        inputs.manipulatorCurrentAmps = manipulatorCurrentSignal.getValueAsDouble();
        inputs.manipulatorControlMode = manipulateMotor.getAppliedControl().getName();

        // LaserCAN sensors
        Measurement firstCoralMeasurement = firstCoralSensor.getMeasurement();
        if (firstCoralMeasurement != null) {
            inputs.firstCoralDistanceMm = firstCoralMeasurement.distance_mm;
            inputs.firstCoralSensorConnected = true;
        } else {
            inputs.firstCoralSensorConnected = false;
        }

        Measurement secondCoralMeasurement = secondCoralSensor.getMeasurement();
        if (secondCoralMeasurement != null) {
            inputs.secondCoralDistanceMm = secondCoralMeasurement.distance_mm;
            inputs.secondCoralSensorConnected = true;
        } else {
            inputs.secondCoralSensorConnected = false;
        }

        Measurement algaeMeasurement = algaeSensor.getMeasurement();
        if (algaeMeasurement != null) {
            inputs.algaeDistanceMm = algaeMeasurement.distance_mm;
            inputs.algaeStatus = algaeMeasurement.status;
            inputs.algaeAmbient = algaeMeasurement.ambient;
            inputs.algaeSensorConnected = true;
        } else {
            inputs.algaeSensorConnected = false;
        }
    }

    @Override
    public void setPivotPosition(double rotations) {
        pivotControl.Position = rotations + pivotOffset;
        pivotMotor.setControl(pivotControl);
    }

    @Override
    public void setPivotCoast() {
        pivotMotor.setControl(new CoastOut());
    }

    @Override
    public void setManipulatorDutyCycle(double dutyCycle) {
        manipulateMotor.set(dutyCycle);
    }

    @Override
    public void setManipulatorBrake() {
        manipulateMotor.setControl(new StaticBrake());
    }

    @Override
    public void setManipulatorCoast() {
        manipulateMotor.setControl(new CoastOut());
    }

    @Override
    public void syncPivotFromAbsoluteEncoder() {
        pivotMotor.setPosition(pivotEncoder.get());
    }

    /**
     * Increase the pivot offset (used for fine-tuning during match and should not be relied upon)
     */
    public void increaseOffset() {
        pivotOffset -= 0.01;
    }

    /**
     * Decrease the pivot offset (used for fine-tuning during match and should not be relied upon)
     */
    public void decreaseOffset() {
        pivotOffset += 0.01;
    }

    public double getOffset() {
        return pivotOffset;
    }
}

