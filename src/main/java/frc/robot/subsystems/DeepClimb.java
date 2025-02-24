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
   
    //this is to use the absolute encoder mode on the through bore encoders.
    //We should be using the absolute mode since the climbing motor won't rotate more than once.
    //To use it as an Absolute Encoder however, we do have to connect it to the DIO poet in the RoboRIO. https://www.chiefdelphi.com/t/rev-encoder-absolute-mode-code-examples/425761
    private final DigitalInput encoderInput;
    private final DutyCycleEncoder climbEncoder; 

    public DeepClimb(){
        climbMotor = new TalonFX(Constants.ClimbConstants.climbMotorID);
        chuteMotor = new TalonFX(Constants.ClimbConstants.chuteMotorID);
        
        encoderInput = new DigitalInput(0);
        climbEncoder = new DutyCycleEncoder(encoderInput);
        


    }
 
     public void turnToAngle() {
         double targetRevolutions = (103 / 360.0) * 200;
         // create a Motion Magic request, voltage output
        final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

        // set target position
        chuteMotor.setControl(m_request.withPosition(targetRevolutions));
     }
    public void setStowSpeed(double chuteSpeed){
        chuteMotor.set(chuteSpeed);

    }
    public boolean isStowed(){
        if (chuteMotor.get() == (103 / 360.0) * 200){
         return true;
        }
        else{
            return false;
        }
    }
     public double getClimbAngle(){
        return climbEncoder.get()/360.0*200;
     }
        public void climbToAngle(double angle){
            double targetRevolutions = (angle / 360.0) * 200;
            // create a Motion Magic request, voltage output
           final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
           // set target position
           climbMotor.setControl(m_request.withPosition(targetRevolutions));
    
        }
     public void setClimbSpeed(double climbSpeed){
         chuteMotor.set(climbSpeed);

     }
        public boolean hasClimbed(){
            if (climbEncoder.get() == (105 / 360.0) * 200){
            return true;
            }
            else{
                return false;
            }
        }

}
