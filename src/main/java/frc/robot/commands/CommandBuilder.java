package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MechanismPosition;
import frc.robot.commands.components.FunnelToPosition;
import frc.robot.commands.components.EndEffectorToPosition;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;

public class CommandBuilder {
    public static Command deploy(Climber climber, EndEffector endEffector, Elevator elevator) {
        return new FunnelToPosition(climber, MechanismPosition.DEPLOY)
            .andThen(new EndEffectorToPosition(endEffector, MechanismPosition.DEPLOY))
            .andThen(new FunnelToPosition(climber, MechanismPosition.REST));
    }

}
