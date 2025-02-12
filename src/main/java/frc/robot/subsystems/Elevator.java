package frc.robot.subsystems;

import java.util.OptionalDouble;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    private final DigitalInput lowerLimitSwitch;
    private boolean reachedBottom;
    private boolean reachedTarget;
    // private final TalonFXConfiguration elevatorConfiguration;

    public Elevator() {
        leftMotor = new TalonFX(Constants.ElevatorConstants.leftMotorCANId);
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = 0.02;
        slot0Configs.kI = 0.0015;
        leftMotor.getConfigurator().apply(slot0Configs);
        rightMotor = new TalonFX(Constants.ElevatorConstants.rightMotorCANId);
        lowerLimitSwitch = new DigitalInput(Constants.ElevatorConstants.lowerLimitSwitchDIOPort);
        reachedBottom = false;

        var currentConfigs = new MotorOutputConfigs();
        currentConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        leftMotor.getConfigurator().apply(currentConfigs);


        rightMotor.setControl(new Follower(ElevatorConstants.leftMotorCANId, true));
    }

    public void setSpeed(double val) {
        leftMotor.set(val);
    }

    public void stop() {
        leftMotor.set(0);
    }

    public boolean getLimitSwitch() {
        return !lowerLimitSwitch.get();
    }

    @Override
    public void periodic() {
        reachedBottom = getLimitSwitch();
        SmartDashboard.putString("Control Mode", leftMotor.getAppliedControl().getName());
        if (leftMotor.getAppliedControl().getName().equals("PositionDutyCycle")) {
            SmartDashboard.putString("Target Position", leftMotor.getAppliedControl().getControlInfo().get("Position"));
        }
        SmartDashboard.putNumber("Current Position", leftMotor.getPosition().getValueAsDouble());
    }

    public void resetEncoders() {
        leftMotor.setPosition(0);
        rightMotor.setPosition(0);
    }

    public void setControl(ControlRequest control) {
        leftMotor.setControl(control);
    }

    public void setNeutralMode(NeutralModeValue value) {
        leftMotor.setNeutralMode(value);
    }

    public double getVelocity() {
        return leftMotor.getVelocity().getValueAsDouble();
    }

    public OptionalDouble getTargetPosition() {
        if(leftMotor.getAppliedControl().getName().equals("PositionDutyCycle")) {
            return OptionalDouble.of(Double.parseDouble(leftMotor.getAppliedControl().getControlInfo().get("Position")));
        }
        return OptionalDouble.empty();
    }

    public boolean hasReachedTarget() {
        if (leftMotor.getAppliedControl().getName().equals("PositionDutyCycle")) {
            return getVelocity() == 0 && Math.abs(leftMotor.getPosition().getValueAsDouble() - getTargetPosition().getAsDouble()) < Constants.ElevatorConstants.targetThreshold;
        }
        return false;
    }

    public void setBrake() {
        setControl(new StaticBrake());
    }

    public void setCoast() {
        setControl(new CoastOut());
    }

}
