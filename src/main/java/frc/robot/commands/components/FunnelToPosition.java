package frc.robot.commands.components;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MechanismPosition;
import frc.robot.subsystems.Climber;
import frc.robot.Constants.ClimberConstants;

public class FunnelToPosition extends Command {

  private Climber climber;
  Rotation2d targetPosition;

  public FunnelToPosition(Climber climber, MechanismPosition position) {
    this.climber = climber;
    addRequirements(climber);
    switch (position) {
        case INTAKE: targetPosition = Rotation2d.fromRotations(0); break;
        case DEPLOY: targetPosition = ClimberConstants.deployPosition; break;
        case CLIMB: targetPosition = ClimberConstants.climbPosition; break;
        case REST: targetPosition = ClimberConstants.restPosition; break;
        default: targetPosition = null;
    }
  }

  @Override
  public void initialize() {
    if (targetPosition != null) {
        climber.setFunnelPosition(targetPosition);
    } else {
        climber.setFunnelCoast();
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
    return climber.funnelHasReachedTarget();
  }
}