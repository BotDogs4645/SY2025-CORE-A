package frc.robot.commands.components;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MechanismPosition;
import frc.robot.subsystems.Chute;
import frc.robot.Constants.ChuteConstants;

public class ChuteToPosition extends Command {

  private Chute chute;
  Rotation2d targetPosition;

  public ChuteToPosition(Chute chute, MechanismPosition position) {
    this.chute = chute;
    addRequirements(chute);
    switch (position) {
        case INTAKE: targetPosition = ChuteConstants.intakePosition; break;
        case DEPLOY: targetPosition = ChuteConstants.deployPosition; break;
        case CLIMB: targetPosition = ChuteConstants.climbPosition; break;
        case REST: targetPosition = ChuteConstants.restPosition; break;
        default: targetPosition = null;
    }
  }

  @Override
  public void initialize() {
    if (targetPosition != null) {
      chute.setPosition(targetPosition);
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
    return chute.hasReachedTarget();
  }
}