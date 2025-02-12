package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class DeepClimb extends SubsystemBase {
    private final TalonFX climbMotor;
    private final TalonFX chuteMotor;
    private final TalonFXConfiguration talonFXConfigs;
    //this is to use the absolute encoder mode on the through bore encoders.
    //We should be using the absolute mode since the climbing motor won't rotate more than once.
    //To use it as an Absolute Encoder however, we do have to connect it to the DIO poet in the RoboRIO. https://www.chiefdelphi.com/t/rev-encoder-absolute-mode-code-examples/425761
    private final DigitalInput encoderInput;
    private final DutyCycleEncoder climbEncoder; 

    public DeepClimb(){
        climbMotor = new TalonFX(0);
        chuteMotor = new TalonFX(9);
        talonFXConfigs = new TalonFXConfiguration();
        encoderInput = new DigitalInput(0);
        climbEncoder = new DutyCycleEncoder(encoderInput);
        // Set neutral mode
        chuteMotor.setNeutralMode(NeutralModeValue.Brake);

        // set slot 0 gains
        Slot0Configs slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

        // set Motion Magic settings
        MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

        chuteMotor.getConfigurator().apply(talonFXConfigs);


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