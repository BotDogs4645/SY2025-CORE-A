package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.EndEffector;

public class CommandBuilder {

    EndEffector endEffector;

    public Command outputCoral(EndEffector endEffector, int level) {
        // move elevator and stuff

        this.endEffector = endEffector;

        return Commands.run(() -> {
            endEffector.setPivotPosition(level);
        });
    }

}
