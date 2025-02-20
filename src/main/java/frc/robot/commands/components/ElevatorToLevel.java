package frc.robot.commands.components;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class ElevatorToLevel extends Command {

  private Elevator elevator;
  Distance targetPosition;

  public ElevatorToLevel(Elevator elevator, int level) {
    this.elevator = elevator;
    addRequirements(elevator);
    targetPosition = getPosition(level);
  }

  private Distance getPosition(int level) {
    switch (level) {
      case 1: return ElevatorConstants.Heights.level1;
      case 2: return ElevatorConstants.Heights.level2;
      case 3: return ElevatorConstants.Heights.level3;
      case 4: return ElevatorConstants.Heights.level4;
      default: return null;
    }
  }

  @Override
  public void initialize() {
    if (targetPosition != null) {
      elevator.setTarget(targetPosition);
    }
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return elevator.hasReachedTarget();
  }
}