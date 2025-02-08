package frc.robot.commands.components;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DeepClimb;

public class Climb extends Command{
    private DeepClimb deepClimb;

    public Climb(DeepClimb deepClimb, double climbSpeed){
        this.deepClimb = deepClimb;

        addRequirements(deepClimb);
    }
    @ Override

    public void initialize(){
    }
    @ Override

    public void execute(){
        deepClimb.setClimbSpeed(Constants.ClimbConstants.climbSpeed);
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