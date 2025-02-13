package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DeepClimb;
import frc.robot.commands.components.Climb;
import frc.robot.commands.components.Stow;
public class CommandBuilder {
    public static Command StowChute(DeepClimb deepClimb){
        return new Stow(deepClimb, Constants.ClimbConstants.chuteSpeed)
        .alongWith (new WaitCommand(0.5))
        .andThen(() -> {deepClimb.setStowSpeed(0.15);}, deepClimb)
        .andThen(new WaitCommand(0.15))
        .andThen(() -> {deepClimb.setStowSpeed(0);}, deepClimb);
    }
    public static Command ClimbDeepCage(DeepClimb deepClimb){
        return new Climb(deepClimb, Constants.ClimbConstants.climbSpeed)
        .andThen(() -> {deepClimb.setClimbSpeed(0.15);}, deepClimb)
        .andThen(new WaitCommand(0.15))
        .andThen(() -> {deepClimb.setClimbSpeed(0);}, deepClimb);
    }
}
