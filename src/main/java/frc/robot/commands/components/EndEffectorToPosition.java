package frc.robot.commands.components;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MechanismPosition;
import frc.robot.subsystems.EndEffector;

public class EndEffectorToPosition extends Command {

  private final EndEffector endEffector;
  final Rotation2d targetPosition;
  final MechanismPosition mechanismPosition;

  public EndEffectorToPosition(EndEffector endEffector, MechanismPosition position) {
    this.endEffector = endEffector;
    addRequirements(endEffector);
    targetPosition = position.pivotPosition;
    mechanismPosition = position;
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