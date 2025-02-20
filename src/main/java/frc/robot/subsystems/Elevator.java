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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;

    public Elevator() {
        leftMotor = new TalonFX(Constants.ElevatorConstants.leftMotorCANId);
        rightMotor = new TalonFX(Constants.ElevatorConstants.rightMotorCANId);

        leftMotor.getConfigurator().apply(Constants.ElevatorConstants.pidConfigs);

        MotorOutputConfigs leftConfigs = new MotorOutputConfigs();
        leftConfigs.Inverted = InvertedValue.Clockwise_Positive;
        leftMotor.getConfigurator().apply(leftConfigs);

        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), true));
    }

    public void setSpeed(double val) {
        leftMotor.set(val);
    }

    public double getPosition() {
        return leftMotor.getPosition().getValueAsDouble();
    }

    public void stop() {
        leftMotor.set(0);
    }

    public void setPosition(double position) {
        leftMotor.setPosition(position);
        rightMotor.setPosition(position);
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Left Control Mode", leftMotor.getAppliedControl().getName());
        SmartDashboard.putString("Right Control Mode", rightMotor.getAppliedControl().getName());

        if (leftMotor.getAppliedControl().getName().equals("PositionDutyCycle")) {
            SmartDashboard.putString("Target Position", leftMotor.getAppliedControl().getControlInfo().get("Position"));
        }
        SmartDashboard.putNumber("Current Position Left", leftMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Current Position Right", rightMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Current Velocity Left", leftMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Current Velocity Right", rightMotor.getVelocity().getValueAsDouble());
    }

    public void resetEncoders() {
        setPosition(0);
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
