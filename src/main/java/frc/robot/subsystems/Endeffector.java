package frc.robot.subsystems;

import java.util.Queue;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.AbsoluteEncoder;
 

public class Endeffector extends SubsystemBase{
    private TalonFX mainMotor;
    private TalonFX pivotMotor;
    private AbsoluteEncoder pivotEncoder;
    private DigitalInput input;
    private TalonFXConfiguration talonConfig;

    // https://www.chiefdelphi.com/t/using-rev-through-bore-encoder-as-zeroing-encoder-on-swerve-drive/428855/2

    public void endeffector() {
        this.mainMotor = new TalonFX(0);
        this.pivotMotor = new TalonFX(0);
        DigitalInput input = new DigitalInput(1);
        DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(input);

        // should be abs rotation on startup
        Rotation2d rotationInitial = new Rotation2d(pivotEncoder.get());
        talonConfig = new TalonFXConfiguration().withFeedback(
            new FeedbackConfigs().withSensorToMechanismRatio(Constants.blah);
        );
        pivotMotor.getConfigurator().apply(talonConfig);
        pivotMotor.setPosition(rotationInitial.getRotations());
    }

    // False for coral
    // True for Algae
    // Spins direction while being called
    public void setSpin(double speed){
        mainMotor.set(speed);
    }

    public void setPivotPosition(int level){
        double level1 = 123123;
        double level2 = 123123;
        double level3 = 123123;
        double level4 = 123123;

        double cur = pivotEncoder.getPosition();
        SmartDashboard.putNumber("pivotEncoderRotations", cur);

        double allowedError = 12345678; // maybe put in constants
        
        switch(level) {
            case 1:
                if(cur < level1) {
                    pivotMotor
                } else if(cur > level1) {
                    pivotMotor.set(-100000000);
                }
            case 2:
                if(cur < level2) {
                    pivotMotor.set(100000000);
                } else if(cur > level2) {
                    pivotMotor.set(-100000000);
                }
            case 3:
                if(cur < level3) {
                    pivotMotor.set(100000000);
                } else if(cur > level3) {
                    pivotMotor.set(-100000000);
                }
            case 4:
                if(cur < level4) {
                    pivotMotor.set(100000000);
                } else if(cur > level4) {
                    pivotMotor.set(-100000000);
                }
        }
    }

    public void rotate(double speed) {
        pivotMotor.set(speed);
    }

    public void stopSpin() {
        mainMotor.set(0);
    }

    public void stopPivot() {
        pivotMotor.set(0);
    }
}
