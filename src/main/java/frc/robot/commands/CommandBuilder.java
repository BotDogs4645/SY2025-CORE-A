package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Endeffector;

public class CommandBuilder {
    public class EndEffector {

        public Command outputCoral(Endeffector endEffector, int level) {
            //move elevator and stuff

            return Commands.run(

                endEffector.setPivotPosition(level);


            )
        }
    }
}
