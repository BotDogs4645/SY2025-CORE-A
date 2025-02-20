package frc.robot.commands.components;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MechanismPosition;
import frc.robot.subsystems.EndEffector;

public class EndEffectorToPosition extends Command {

  private EndEffector endEffector;
  Rotation2d targetPosition;

  public EndEffectorToPosition(EndEffector endEffector, MechanismPosition position) {
    this.endEffector = endEffector;
    addRequirements(endEffector);
    targetPosition = position.pivotPosition;
  }

  @Override
  public void initialize() {
    if (targetPosition != null) {
      endEffector.setPivotPosition(targetPosition);
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
    return endEffector.hasReachedTarget();
  }
}