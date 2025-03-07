package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ChuteConstants;

public class Chute extends SubsystemBase {
    
    private final TalonFX chuteMotor;

    private final DutyCycleEncoder chuteEncoder;

    private final PositionVoltage chuteControl;

    public Chute() {
        chuteMotor = new TalonFX(ChuteConstants.motorID);

        chuteEncoder = new DutyCycleEncoder(ChuteConstants.encoderDioPort);

        var chuteMotorConfig = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                .withKP(ChuteConstants.KP)
                .withKI(ChuteConstants.KI)
                .withKD(ChuteConstants.KD)
            )       
            .withFeedback(new FeedbackConfigs()
                .withSensorToMechanismRatio(ChuteConstants.gearRatio)
            );

        chuteMotor.getConfigurator().apply(chuteMotorConfig);
        chuteMotor.setPosition(chuteEncoder.get());

        chuteControl = new PositionVoltage(chuteMotor.getPosition().getValueAsDouble());
        //chuteMotor.setControl(chuteControl);
        chuteMotor.setControl(new CoastOut());
    }

    public void setPosition(Rotation2d position) {
        chuteControl.Position = position.getRotations();
        chuteMotor.setControl(chuteControl);
    }

    public double getPosition() {
        return chuteMotor.getPosition().getValueAsDouble();
    }

    public double getVelocity() {
        return chuteMotor.getVelocity().getValueAsDouble();
    }

    public double getSetpoint() {
        return chuteControl.Position;
    }

    public boolean hasReachedTarget() {
        return chuteMotor.getAppliedControl() == chuteControl
            && Math.abs(getPosition() - getSetpoint()) <= ChuteConstants.rotationThreshold.getRotations()
            && Math.abs(getVelocity()) <= ChuteConstants.velocityThreshold.in(RadiansPerSecond);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Chute/position", getPosition());
        Logger.recordOutput("Chute/control", chuteMotor.getAppliedControl().getName());
        Logger.recordOutput("Chute/setpoint", getSetpoint());
        Logger.recordOutput("Chute/velocity", getVelocity());
        Logger.recordOutput("Chute/voltageOut", chuteMotor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput("Chute/done", hasReachedTarget());
    }
}