package frc.robot.commands.components;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class ElevatorDown extends Command {

    private Elevator elevator;
        private CommandXboxController controller;


    public ElevatorDown(Elevator elevator, CommandXboxController controller) {
        this.elevator = elevator;
        addRequirements(elevator);
        this.controller = controller;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
      if(elevator.getPosition() > 7) {
        elevator.setSpeed(Constants.ElevatorConstants.elevatorDownSpeed);
        controller.setRumble(RumbleType.kLeftRumble, 1);
      }
      else {
        elevator.setSpeed(0);
        controller.setRumble(RumbleType.kLeftRumble, 0);
      }
    }

    @Override
    public void end(boolean interrupted) {
      // elevator.setBrake();
      elevator.setSpeed(0);
      controller.setRumble(RumbleType.kBothRumble, 0);
    }

    

    @Override
    public boolean isFinished() {
      return false;
    }



}