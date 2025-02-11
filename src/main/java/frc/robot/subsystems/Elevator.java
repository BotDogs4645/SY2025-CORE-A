package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
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

public class Elevator extends SubsystemBase{
    private final TalonFX leftMotor;
    // private final TalonFX rightMotor;
    private final DigitalInput lowerLimitSwitch;
    private boolean reachedBottom;
    // private final TalonFXConfiguration elevatorConfiguration;

    public Elevator() {
        leftMotor = new TalonFX(Constants.ElevatorConstants.leftMotorCANId);
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = 0.05;
        leftMotor.getConfigurator().apply(slot0Configs);
        // rightMotor = new TalonFX(Constants.ElevatorConstants.rightMotorCANId);
        lowerLimitSwitch = new DigitalInput(Constants.ElevatorConstants.lowerLimitSwitchDIOPort);
        reachedBottom = false;

        var currentConfigs = new MotorOutputConfigs();
        currentConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        leftMotor.getConfigurator().apply(currentConfigs);

        currentConfigs.Inverted = InvertedValue.Clockwise_Positive;
        // rightMotor.getConfigurator().apply(currentConfigs);
    }

    public void setSpeed(double val) {
        // rightMotor.set(val);
        leftMotor.set(val);
    }

    public void stop() {
        leftMotor.set(0);
        // rightMotor.set(0);
    }
    
    public boolean getLimitSwitch() {
        return !lowerLimitSwitch.get();
    }

    @Override
    public void periodic() {
        reachedBottom = getLimitSwitch();
        SmartDashboard.putBoolean("limitswitch", reachedBottom);
        SmartDashboard.putNumber("encoder", leftMotor.getPosition().getValueAsDouble());
    } 

    public void resetEncoders() {
        leftMotor.setPosition(0);
        // rightMotor.setPosition(0);
    }

    public void setControl(ControlRequest control) {
        leftMotor.setControl(control);
    }

    public void enableBrakemode() {
        leftMotor.setNeutralMode(NeutralModeValue.Brake);
        // rightMotor.setNeutralMode(NeutralModeValue.Brake);
    }
    public void enableCoastMode() {
        leftMotor.setNeutralMode(NeutralModeValue.Coast);
        // rightMotor.setNeutralMode(NeutralModeValue.Coast);
    }



}
