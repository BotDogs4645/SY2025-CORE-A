package frc.robot.commands.components;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DeepClimb;

public class Stow extends Command{
    private DeepClimb deepClimb;

    public Stow(DeepClimb deepClimb, double chuteSpeed){
        this.deepClimb = deepClimb;

        addRequirements(deepClimb);
    }
    @ Override

    public void initialize(){
    }
    @ Override

    public void execute(){
        deepClimb.stowToAngle();
    }
    @ Override

    public void end(boolean interrupted){
        deepClimb.setStowSpeed(0);
    }

    @ Override

    public boolean isFinished(){
        return deepClimb.isStowed();
    }  
}
