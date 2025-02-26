package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DeepClimb extends SubsystemBase {
    private final TalonFX climbMotor;
    private final TalonFX chuteMotor;

    // this is to use the absolute encoder mode on the through bore encoders.
    // We should be using the absolute mode since the climbing motor won't rotate
    // more than once.
    // To use it as an Absolute Encoder however, we do have to connect it to the DIO
    // poet in the RoboRIO.
    // https://www.chiefdelphi.com/t/rev-encoder-absolute-mode-code-examples/425761
    private final DigitalInput encoderInput;
    private final DutyCycleEncoder climbEncoder;

    public DeepClimb() {
        climbMotor = new TalonFX(Constants.ClimbConstants.climbMotorID);
        chuteMotor = new TalonFX(Constants.ClimbConstants.chuteMotorID);

        encoderInput = new DigitalInput(0);
        climbEncoder = new DutyCycleEncoder(encoderInput);
        chuteMotor.getConfigurator().apply(Constants.ClimbConstants.talonFXConfigs);

    }

    public void stowToAngle() {
        double targetRotations = (Constants.ClimbConstants.angleToStow / 360.0) * Constants.ClimbConstants.stowGearRatio;
        // create a Motion Magic request, voltage output
        final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

        // set target position
        chuteMotor.setControl(m_request.withPosition(targetRotations));
    }

    public void setStowSpeed(double chuteSpeed) {
        chuteMotor.set(chuteSpeed);

    }

    public boolean isStowed() {
        if (Math.abs(chuteMotor.get() - Constants.ClimbConstants.stowTopPoint) <= Constants.ClimbConstants.stowDeadband 
        && chuteMotor.getVelocity().getValueAsDouble() == 0){
            return true;
        } else {
            return false;
        }
    }

    public double getClimbAngle() {
        return climbEncoder.get() / 360.0 * Constants.ClimbConstants.stowGearRatio;
    }

    public void climbToAngle(double angle) {
        double targetRotations = (angle / 360.0) * Constants.ClimbConstants.climbGearRatio;
        // create a Motion Magic request, voltage output
        final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
        // set target position
        climbMotor.setControl(m_request.withPosition(targetRotations));

    }

    public void setClimbSpeed(double climbSpeed) {
        chuteMotor.set(climbSpeed);

    }

    public boolean hasClimbed() {
        if (Math.abs(climbEncoder.get() - Constants.ClimbConstants.climbBottomEndStop) <= Constants.ClimbConstants.climbDeadband 
        && climbMotor.getVelocity().getValueAsDouble() == 0){
            return true;
        } else {
            return false;
        }
    }

}
