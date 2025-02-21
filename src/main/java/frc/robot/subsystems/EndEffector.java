package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
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

    public EndEffector() {
        firstCoralSensor = new LaserCan(EndEffectorConstants.firstCoralSensorID);
        secondCoralSensor = new LaserCan(EndEffectorConstants.secondCoralSensorID);
        algaeSensor = new LaserCan(EndEffectorConstants.algaeSensorID);

        manipulateMotor = new TalonFX(EndEffectorConstants.manipulateMotorID);
        pivotMotor = new TalonFX(EndEffectorConstants.pivotMotorID);
        pivotEncoder = new DutyCycleEncoder(EndEffectorConstants.encoderDioPort);

        Rotation2d rotationInitial = Rotation2d.fromRotations(pivotEncoder.get());
        var pivotConfig = new TalonFXConfiguration()
            .withFeedback(new FeedbackConfigs()
                .withSensorToMechanismRatio(EndEffectorConstants.gearRatio)
            ).withSlot0(new Slot0Configs()
                //.withKS(0)
                //.withKV(0)
                //.withKA(0)
                .withKP(EndEffectorConstants.KP)
                .withKI(EndEffectorConstants.KI)
                .withKD(EndEffectorConstants.KD)
            );//.withMotionMagic(new MotionMagicConfigs()
            //    .withMotionMagicCruiseVelocity(80 / EndEffectorConstants.gearRatio)
            //    .withMotionMagicAcceleration(160 / EndEffectorConstants.gearRatio)
            //    .withMotionMagicJerk(1600 / EndEffectorConstants.gearRatio)
            //);

        pivotMotor.getConfigurator().apply(pivotConfig);
        
        Rotation2d offset = rotationInitial.minus(EndEffectorConstants.endoderOffset);
        pivotControl = new MotionMagicVoltage(offset.getRotations());
        pivotMotor.setPosition(offset.getRotations());

        pivotMotor.setControl(pivotControl);
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
        return firstCoralSensor.getMeasurement().distance_mm <= EndEffectorConstants.coralThreshold.in(Millimeters);
    }

    public boolean secondCoralSensorTripped() {
        return secondCoralSensor.getMeasurement().distance_mm  <= EndEffectorConstants.coralThreshold.in(Millimeters);
    }

    public boolean algaeSensorTripped() {
        return algaeSensor.getMeasurement().distance_mm  <= EndEffectorConstants.algaeThreshold.in(Millimeters);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("EndEffector/pivotPosition", getPivotPosition());
        Logger.recordOutput("EndEffector/pivotSetpoint", getPivotTargetPosition());
        Logger.recordOutput("EndEffector/pivotVelocity", getPivotVelocity());
        Logger.recordOutput("EndEffector/pivotControl", pivotMotor.getAppliedControl().getName());
        Logger.recordOutput("EndEffector/pivotDone", hasReachedTarget());
        Logger.recordOutput("EndEffector/encoderPosition", pivotEncoder.get());
    }
}