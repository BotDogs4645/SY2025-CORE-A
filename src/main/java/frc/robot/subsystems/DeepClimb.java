package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DeepClimb extends SubsystemBase {
    private final TalonFX climbMotor;
    private final TalonFX chuteMotor;
    //this is to use the absolute encoder mode on the through bore encoders.
    //We should be using the absolute mode since the climbing motor won't rotate more than once.
    //To use it as an Absolute Encoder however, we do have to connect it to the DIO poet in the RoboRIO. https://www.chiefdelphi.com/t/rev-encoder-absolute-mode-code-examples/425761
    private final DigitalInput encoderInput;
    private final DutyCycleEncoder climbEncoder; 
    public DeepClimb(){
        climbMotor = new TalonFX(0);
        chuteMotor = new TalonFX(0);
        encoderInput = new DigitalInput(0);
        climbEncoder = new DutyCycleEncoder(encoderInput);
    }
    public void climb (double climbSpeed, double chuteSpeed){
        while (chuteMotor.getPosition().getValueAsDouble() < 103.0 ){ //the value is in angles
            chuteMotor.set(chuteSpeed);
        }
        while (climbEncoder.get() < 0.625){ //Need to test this value (the value is in rotations)
            climbMotor.set(climbSpeed);
        }
    }
}
