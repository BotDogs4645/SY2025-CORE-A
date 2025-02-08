package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DeepClimb;

public class ClimbDeepCage extends Command{
    private DeepClimb deepClimb;

    public ClimbDeepCage(DeepClimb deepClimb, double climbSpeed){
        this.deepClimb = deepClimb;

        addRequirements(deepClimb);
    }
    @ Override

    public void initialize(){
    }
    @ Override

    public void execute(){
        if (deepClimb.getClimbAngle() < 225.0){
            deepClimb.setClimbSpeed(Constants.ClimbConstants.climbSpeed);
        }else{
            deepClimb.setClimbSpeed(0);
        }
    }
    @ Override

    public void end(boolean interrupted){
        deepClimb.setClimbSpeed(0);
    }

    @ Override

    public boolean isFinished(){
        return true;
    }

}