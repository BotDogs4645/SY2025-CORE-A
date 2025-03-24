package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
    
    private final TalonFX climbMotor;

    private final DutyCycleEncoder climbEncoder;

    public Climber() {
        climbMotor = new TalonFX(ClimberConstants.motorID);

        climbEncoder = new DutyCycleEncoder(ClimberConstants.encoderDioPort);

        var climbMotorConfig = new TalonFXConfiguration()
            .withFeedback(new FeedbackConfigs()
            .withSensorToMechanismRatio(ClimberConstants.gearRatio)
        );

        climbMotor.getConfigurator().apply(climbMotorConfig);
        climbMotor.setPosition(climbEncoder.get());

        climbMotor.setControl(new StaticBrake());
    }

    public double getPosition() {
        return climbMotor.getPosition().getValueAsDouble();
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Climber/position", getPosition());
        Logger.recordOutput("Climber/encoderPosition", climbEncoder.get());
        Logger.recordOutput("Climber/voltageOut", climbMotor.getMotorVoltage().getValueAsDouble());
    }
}