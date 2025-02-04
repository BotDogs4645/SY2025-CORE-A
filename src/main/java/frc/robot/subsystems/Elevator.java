package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase{
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    private final DigitalInput lowerLimitSwitch;
    private boolean reachedBottom;
    // private final TalonFXConfiguration elevatorConfiguration;

    public Elevator() {
        leftMotor = new TalonFX(Constants.ElevatorConstants.leftMotorCANId);
        rightMotor = new TalonFX(Constants.ElevatorConstants.rightMotorCANId);
        lowerLimitSwitch = new DigitalInput(Constants.ElevatorConstants.lowerLimitSwitchDIOPort);
        reachedBottom = false;

        rightMotor.setInverted(true);
    }

    public void setSpeed(double val) {
        rightMotor.set(val);
        leftMotor.set(val);
    }

    public void stop() {
        leftMotor.set(0);
        rightMotor.set(0);
    }
    
    public boolean getLimitSwitch() {
        return !lowerLimitSwitch.get();
    }

    @Override
    public void periodic() {
        reachedBottom = getLimitSwitch();
    } 


}
