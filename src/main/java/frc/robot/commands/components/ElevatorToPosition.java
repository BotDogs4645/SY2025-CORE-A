package frc.robot.commands.components;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MechanismPosition;
import frc.robot.subsystems.Elevator;

public class ElevatorToPosition extends Command {

  private final Elevator elevator;
  final Distance targetPosition;

  public ElevatorToPosition(Elevator elevator, MechanismPosition position) {
    this.elevator = elevator;
    addRequirements(elevator);
    targetPosition = position.elevatorPosition;
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