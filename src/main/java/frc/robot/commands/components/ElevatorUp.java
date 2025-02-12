package frc.robot.commands.components;

import java.util.ResourceBundle.Control;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class ElevatorUp extends Command {

    private Elevator elevator;
    private CommandXboxController controller;

    public ElevatorUp(Elevator elevator, CommandXboxController controller) {
        this.elevator = elevator;
        addRequirements(elevator);
        this.controller = controller;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        elevator.setSpeed(Constants.ElevatorConstants.elevatorUpSpeed);
        controller.setRumble(RumbleType.kRightRumble, 1);
        
    }

    @Override
    public void end(boolean interrupted) {
      elevator.setBrake();
      controller.setRumble(RumbleType.kBothRumble, 0);
    }

    

    @Override
    public boolean isFinished() {
      return false;
    }



}