package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

    public static Command intake(EndEffector endEffector) {
        return new Command() {
            @Override
            public void initialize() {
                endEffector.setWheelDutyCycle(0.2);
            }

            @Override
            public void execute() {
                if (endEffector.firstCoralSensorTripped()) {
                    endEffector.setWheelDutyCycle(0.05);
                }
            }

            @Override
            public void end(boolean interrupted) {
                endEffector.setWheelDutyCycle(0);
            }

            @Override 
            public boolean isFinished() {
                return endEffector.secondCoralSensorTripped();
            }
        };
    } 

    public static Command spit(EndEffector endEffector) {
        return new Command() {
            @Override
            public void initialize() {
                endEffector.setWheelDutyCycle(0.2);
            }

            @Override
            public void execute() {
            }

            @Override
            public void end(boolean interrupted) {
                endEffector.setWheelDutyCycle(0);
            }

            @Override 
            public boolean isFinished() {
                return false;
            }
        };
    } 

}
