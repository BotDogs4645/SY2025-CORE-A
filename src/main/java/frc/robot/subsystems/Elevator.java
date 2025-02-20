package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

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

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;

    private final PositionDutyCycle positionControl;

    public Elevator() {
        leftMotor = new TalonFX(ElevatorConstants.leftMotorCANId, "*");
        rightMotor = new TalonFX(ElevatorConstants.rightMotorCANId, "*");

        var config = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                .withKP(ElevatorConstants.KP)
                .withKI(ElevatorConstants.KI)
                .withKD(ElevatorConstants.KD)
            ).withMotorOutput(new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
            );

        leftMotor.getConfigurator().apply(config);
        rightMotor.getConfigurator().apply(config);

        positionControl = new PositionDutyCycle(0);
        rightMotor.setControl(new Follower(ElevatorConstants.leftMotorCANId, true));
        leftMotor.setControl(positionControl);
    }

    @Override
    public void periodic() {
    }

    public void resetEncoders() {
        leftMotor.setPosition(0);
        rightMotor.setPosition(0);
    }

    public double getPosition() {
        return leftMotor.getPosition().getValueAsDouble() * ElevatorConstants.metersPerRotation;
    }

    
    public double getVelocity() {
        return leftMotor.getVelocity().getValueAsDouble() * ElevatorConstants.metersPerRotation;
    }

    public double getTargetPosition() {
        return positionControl.Position * ElevatorConstants.metersPerRotation;
    }

    public boolean hasReachedTarget() {
        return leftMotor.getAppliedControl() == positionControl
            && Math.abs(getTargetPosition() - getPosition()) <= ElevatorConstants.positionThreshold.in(Meters)
            && Math.abs(getVelocity()) <= ElevatorConstants.velocityThreshold.in(MetersPerSecond);
    }

    public void setBrake(Boolean brake) {
        leftMotor.setControl(brake ? new StaticBrake() : new CoastOut());
    }

    public void setTarget(Distance distance) {
        positionControl.Position = distance.in(Meters) / ElevatorConstants.metersPerRotation;
        leftMotor.setControl(positionControl);
    }

    // TODO: add manual teleop controls for operator 
    public void setDutyCycle(double dutyCycle) {
        leftMotor.setControl(new DutyCycleOut(dutyCycle));
    }
}
