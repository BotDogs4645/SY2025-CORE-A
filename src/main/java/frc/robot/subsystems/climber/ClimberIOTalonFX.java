package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.ClimberConstants;

/**
 * Real hardware implementation of ClimberIO using TalonFX motor.
 */
public class ClimberIOTalonFX implements ClimberIO {
    private final TalonFX climbMotor;
    private final DutyCycleEncoder climbEncoder;

    // status signals for efficient reading
    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Voltage> voltageSignal;
    private final StatusSignal<Current> currentSignal;

    public ClimberIOTalonFX() {
        climbMotor = new TalonFX(ClimberConstants.motorID);
        climbEncoder = new DutyCycleEncoder(ClimberConstants.encoderDioPort);

        var climbMotorConfig = new TalonFXConfiguration()
            .withFeedback(new FeedbackConfigs()
                .withSensorToMechanismRatio(ClimberConstants.gearRatio)
            );

        climbMotor.getConfigurator().apply(climbMotorConfig);
        climbMotor.setPosition(climbEncoder.get());
        climbMotor.setControl(new StaticBrake());

        // Cache status signals
        positionSignal = climbMotor.getPosition();
        velocitySignal = climbMotor.getVelocity();
        voltageSignal = climbMotor.getMotorVoltage();
        currentSignal = climbMotor.getStatorCurrent();

        // optimize CAN bus usage
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            positionSignal,
            velocitySignal,
            voltageSignal,
            currentSignal
        );
        climbMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            positionSignal,
            velocitySignal,
            voltageSignal,
            currentSignal
        );

        inputs.positionRotations = positionSignal.getValueAsDouble();
        inputs.velocityRotationsPerSec = velocitySignal.getValueAsDouble();
        inputs.appliedVolts = voltageSignal.getValueAsDouble();
        inputs.currentAmps = currentSignal.getValueAsDouble();
        inputs.controlMode = climbMotor.getAppliedControl().getName();

        inputs.absoluteEncoderPositionRotations = climbEncoder.get();
        inputs.absoluteEncoderConnected = climbEncoder.isConnected();
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        climbMotor.setControl(new DutyCycleOut(dutyCycle));
    }

    @Override
    public void setBrake() {
        climbMotor.setControl(new StaticBrake());
    }

    @Override
    public void syncFromAbsoluteEncoder() {
        climbMotor.setPosition(climbEncoder.get());
    }
}

