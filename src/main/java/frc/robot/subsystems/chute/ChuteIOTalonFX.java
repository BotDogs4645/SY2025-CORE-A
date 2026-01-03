package frc.robot.subsystems.chute;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.ChuteConstants;

/**
 * Real hardware implementation of ChuteIO using TalonFX motor.
 */
public class ChuteIOTalonFX implements ChuteIO {
    private final TalonFX chuteMotor;
    private final DutyCycleEncoder chuteEncoder;
    private final PositionVoltage chuteControl;

    // status signals for efficient reading
    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Voltage> voltageSignal;
    private final StatusSignal<Current> currentSignal;

    public ChuteIOTalonFX() {
        chuteMotor = new TalonFX(ChuteConstants.motorID);
        chuteEncoder = new DutyCycleEncoder(ChuteConstants.encoderDioPort, 1, 0.85);

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

        chuteControl = new PositionVoltage(chuteEncoder.get());
        chuteMotor.setControl(new CoastOut());

        // cache status signals
        positionSignal = chuteMotor.getPosition();
        velocitySignal = chuteMotor.getVelocity();
        voltageSignal = chuteMotor.getMotorVoltage();
        currentSignal = chuteMotor.getStatorCurrent();

        // optimize CAN bus usage
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            positionSignal,
            velocitySignal,
            voltageSignal,
            currentSignal
        );
        chuteMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ChuteIOInputs inputs) {
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
        inputs.controlMode = chuteMotor.getAppliedControl().getName();

        inputs.absoluteEncoderPositionRotations = chuteEncoder.get();
        inputs.absoluteEncoderConnected = chuteEncoder.isConnected();
    }

    @Override
    public void setTargetPosition(double rotations) {
        chuteControl.Position = rotations;
        chuteMotor.setControl(chuteControl);
    }

    @Override
    public void setCoast() {
        chuteMotor.setControl(new CoastOut());
    }

    @Override
    public void syncFromAbsoluteEncoder() {
        chuteMotor.setPosition(chuteEncoder.get());
    }
}

