package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DeepClimb extends SubsystemBase {
    DeepClimb deepClimb = new DeepClimb();
    private final TalonFX climbMotor;
    private final TalonFX chuteMotor;
    public double ChuteAngle;
    //this is to use the absolute encoder mode on the through bore encoders.
    //We should be using the absolute mode since the climbing motor won't rotate more than once.
    //To use it as an Absolute Encoder however, we do have to connect it to the DIO poet in the RoboRIO. https://www.chiefdelphi.com/t/rev-encoder-absolute-mode-code-examples/425761
    private final DigitalInput encoderInput;
    private final DutyCycleEncoder climbEncoder; 
    public DeepClimb(){
        climbMotor = new TalonFX(0);
        chuteMotor = new TalonFX(9);
        encoderInput = new DigitalInput(0);
        climbEncoder = new DutyCycleEncoder(encoderInput);
    }
    public double getChuteAngle() {
        ChuteAngle = chuteMotor.getPosition().getValueAsDouble()/200;
        return ChuteAngle;
    }
    public boolean isStowed() {
        if (ChuteAngle == 103){
            return true;
        }else{
            return false;
        }
    }
    public void resetChuteAngle(){
        chuteMotor.set(0);
        ChuteAngle = 0;
    }
    public void setStowSpeed (double chuteSpeed){
        chuteMotor.set(chuteSpeed);
    }
    public double getClimbAngle() {
        return climbEncoder.get()*360.0;
    }
    public boolean hasClimbed() {
        if (climbEncoder.get()*360 == 225){
            return true;
        }else{
            return false;
        }
    }
    public void setClimbSpeed (double climbSpeed){
        climbMotor.set(climbSpeed);
    }
     @Override
     public void periodic() {
         SmartDashboard.putNumber("Chute Angle", ChuteAngle);
         SmartDashboard.putBoolean("Chute Stowed", deepClimb.isStowed());
     }

}
