package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;;

public class Climber extends SubsystemBase {
    
    private final TalonFX climbMotorA;
    private final TalonFX climbMotorB;
    private final TalonFX funnelMotor; 

    private final DutyCycleEncoder climbEncoder;

    private final PositionVoltage climberControl;
    private final PositionVoltage funnelControl;

    public Climber() {
        climbMotorA = new TalonFX(ClimberConstants.climbMotorAID);
        climbMotorB = new TalonFX(ClimberConstants.climbMotorBID);
        funnelMotor = new TalonFX(ClimberConstants.funnelMotorID);

        climbEncoder = new DutyCycleEncoder(ClimberConstants.encoderDioPort);

        var climbMotorConfig = new TalonFXConfiguration()
            .withFeedback(new FeedbackConfigs()
                .withSensorToMechanismRatio(ClimberConstants.climbGearRatio)
            );
        var funnelMotorConfig = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                .withKP(10)
            )       
            .withFeedback(new FeedbackConfigs()
                .withSensorToMechanismRatio(ClimberConstants.funnelGearRatio)
            );

        climbMotorA.getConfigurator().apply(climbMotorConfig);
        climbMotorB.getConfigurator().apply(climbMotorConfig);
        funnelMotor.getConfigurator().apply(funnelMotorConfig);

        climbMotorA.setPosition(climbEncoder.get());

        funnelControl = new PositionVoltage(funnelMotor.getPosition().getValueAsDouble());
        climberControl = new PositionVoltage(climbMotorA.getPosition().getValueAsDouble());

        climbMotorB.setControl(new Follower(ClimberConstants.climbMotorAID, true));
        climbMotorA.setControl(climberControl);
        funnelMotor.setControl(new CoastOut());
    }

    public void setFunnelPosition(Rotation2d position) {
        funnelControl.Position = position.getRotations();
        funnelMotor.setControl(funnelControl);
    }

    public void setFunnelCoast() {
        funnelMotor.setControl(new CoastOut());
    }

    public double getFunnelPosition() {
        return funnelMotor.getPosition().getValueAsDouble();
    }

    public double getFunnelVelocity() {
        return funnelMotor.getVelocity().getValueAsDouble();
    }

    public double getFunnelTargetPosition() {
        return funnelControl.Position;
    }

    public boolean funnelHasReachedTarget() {
        return funnelMotor.getAppliedControl().getName().equals("CoastOut") ||
            funnelMotor.getAppliedControl() == funnelControl
            && Math.abs(getFunnelTargetPosition() - getFunnelPosition()) <= ClimberConstants.rotationThreshold.getRotations()
            && Math.abs(getFunnelPosition()) <= ClimberConstants.velocityThreshold.in(RadiansPerSecond);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Climber/funnelPosition", getFunnelPosition());
        Logger.recordOutput("Climber/funnelSetpoint", getFunnelTargetPosition());
        Logger.recordOutput("Climber/funnelVelocity", getFunnelVelocity());
        Logger.recordOutput("Climber/funnelControl", funnelMotor.getAppliedControl().getName());
        Logger.recordOutput("Climber/funnelDone", funnelHasReachedTarget());
    }
}