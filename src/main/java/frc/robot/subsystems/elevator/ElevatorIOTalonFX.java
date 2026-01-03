package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.ElevatorConstants;

/**
 * Real hardware implementation of ElevatorIO using TalonFX motors.
 */
public class ElevatorIOTalonFX implements ElevatorIO {
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;

    private final PositionDutyCycle positionControl;

    // status signals for efficient reading
    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Voltage> voltageSignal;
    private final StatusSignal<Current> currentSignal;
    private final StatusSignal<Temperature> tempSignal;
    private final StatusSignal<ReverseLimitValue> reverseLimitSignal;

    public ElevatorIOTalonFX() {
        leftMotor = new TalonFX(ElevatorConstants.leftMotorCANId, "*");
        rightMotor = new TalonFX(ElevatorConstants.rightMotorCANId, "*");

        var config = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                .withKP(ElevatorConstants.KP)
                .withKI(ElevatorConstants.KI)
                .withKD(ElevatorConstants.KD)
            ).withMotorOutput(new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
            );

        var leftConfig = config.withHardwareLimitSwitch(
            new HardwareLimitSwitchConfigs()
                .withReverseLimitEnable(true)
                .withReverseLimitAutosetPositionEnable(true)
                .withReverseLimitAutosetPositionValue(0)
                .withReverseLimitType(ReverseLimitTypeValue.NormallyOpen)
                .withReverseLimitSource(ReverseLimitSourceValue.LimitSwitchPin)
        );

        leftMotor.getConfigurator().apply(leftConfig);
        rightMotor.getConfigurator().apply(config);

        positionControl = new PositionDutyCycle(0);
        rightMotor.setControl(new Follower(ElevatorConstants.leftMotorCANId, true));
        leftMotor.setControl(positionControl);

        // cache status signals
        positionSignal = leftMotor.getPosition();
        velocitySignal = leftMotor.getVelocity();
        voltageSignal = leftMotor.getMotorVoltage();
        currentSignal = leftMotor.getStatorCurrent();
        tempSignal = leftMotor.getDeviceTemp();
        reverseLimitSignal = leftMotor.getReverseLimit();

        // optimize CAN bus usage by setting update frequencies
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            positionSignal,
            velocitySignal,
            voltageSignal,
            currentSignal,
            tempSignal,
            reverseLimitSignal
        );
        leftMotor.optimizeBusUtilization();
        rightMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            positionSignal,
            velocitySignal,
            voltageSignal,
            currentSignal,
            tempSignal,
            reverseLimitSignal
        );

        inputs.positionRotations = positionSignal.getValueAsDouble();
        inputs.velocityRotationsPerSec = velocitySignal.getValueAsDouble();
        inputs.appliedVolts = voltageSignal.getValueAsDouble();
        inputs.currentAmps = currentSignal.getValueAsDouble();
        inputs.tempCelsius = tempSignal.getValueAsDouble();
        inputs.reverseLimitSwitch = reverseLimitSignal.getValue() == ReverseLimitValue.ClosedToGround;
        inputs.controlMode = leftMotor.getAppliedControl().getName();
    }

    @Override
    public void setTargetPosition(double rotations) {
        positionControl.Position = rotations;
        leftMotor.setControl(positionControl);
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        leftMotor.setControl(new DutyCycleOut(dutyCycle));
    }

    @Override
    public void setBrakeMode(boolean brake) {
        leftMotor.setControl(brake ? new StaticBrake() : new CoastOut());
    }

    @Override
    public void resetPosition(double rotations) {
        leftMotor.setPosition(rotations);
    }
}

