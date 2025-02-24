package frc.robot.commands.components;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DeepClimb;

public class Climb extends Command{
    private DeepClimb deepClimb;

    public Climb(DeepClimb deepClimb, double climbSpeed){
        this.deepClimb = deepClimb;

        addRequirements(deepClimb);
    }
    @ Override

    public void initialize(){
        deepClimb.climbToAngle(0);
    }
    @ Override

    public void execute(){
        deepClimb.climbToAngle(105);
    }
    @ Override

    public void end(boolean interrupted){
        deepClimb.setClimbSpeed(0);
    }

    @ Override

    public boolean isFinished(){
        return deepClimb.hasClimbed();
    }  
}