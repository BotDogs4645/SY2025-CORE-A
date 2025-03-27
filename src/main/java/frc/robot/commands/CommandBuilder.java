package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.MechanismPosition;
import frc.robot.commands.components.AutoElevatorToPosition;
import frc.robot.commands.components.ChuteToPosition;
import frc.robot.commands.components.ElevatorToPosition;
import frc.robot.commands.components.EndEffectorComponents;
import frc.robot.commands.components.EndEffectorToPosition;
import frc.robot.subsystems.Chute;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;

public class CommandBuilder {
    public static Command deploy(Chute chute, EndEffector endEffector, Elevator elevator) {
        return new ChuteToPosition(chute, MechanismPosition.DEPLOY)
            .andThen(new EndEffectorToPosition(endEffector, MechanismPosition.DEPLOY))
            .andThen(new ChuteToPosition(chute, MechanismPosition.REST))
            .andThen(new InstantCommand(() -> {endEffector.deployed = true;}));
            
    }

    public static Command toggleDeploy(Chute chute, EndEffector endEffector, Elevator elevator) {
        return new ConditionalCommand(
            new ChuteToPosition(chute, MechanismPosition.DEPLOY)
            .andThen(new ElevatorToPosition(elevator, MechanismPosition.STOW))
            .andThen(new EndEffectorToPosition(endEffector, MechanismPosition.STOW))
            .andThen(new ChuteToPosition(chute, MechanismPosition.STOW))
            .andThen(new InstantCommand(() -> {endEffector.deployed = false;})),
            deploy(chute, endEffector, elevator),
            endEffector::isDeployed
        );
    }

    public static Command intakeOrProcessor(Chute chute, EndEffector endEffector, Elevator elevator) {
        return new ConditionalCommand(
            new ElevatorToPosition(elevator, MechanismPosition.PROCESSOR)
            .andThen(new EndEffectorToPosition(endEffector, MechanismPosition.PROCESSOR)),
            intakeSequence(chute, endEffector, elevator),
            endEffector::algaeSensorTripped
        );
    }

    public static Command toMechanismPosition(EndEffector endEffector, Elevator elevator, MechanismPosition position) {
        return new ParallelCommandGroup(
            new ElevatorToPosition(elevator, position),
            new EndEffectorToPosition(endEffector, position)
        );
    }
    public static Command toAutoMechanismPosition(EndEffector endEffector, Elevator elevator, MechanismPosition position) {
        return new ParallelCommandGroup(
            new AutoElevatorToPosition(elevator, position),
            new EndEffectorToPosition(endEffector, position)
        );
    }

    public static Command intakeSequence(Chute chute, EndEffector endEffector, Elevator elevator) {
        return (toMechanismPosition(endEffector, elevator, MechanismPosition.INTAKE)
        .andThen(
            new ChuteToPosition(chute, MechanismPosition.INTAKE)
        ).andThen(
            EndEffectorComponents.intakeCoral(endEffector)
        )).finallyDo(
            new ChuteToPosition(chute, MechanismPosition.REST)::schedule
        );
    }
    public static Command autoIntakeSequence(Chute chute, EndEffector endEffector, Elevator elevator) {
        return (toMechanismPosition(endEffector, elevator, MechanismPosition.AUTO_INTAKE)
                .andThen(
                        new ChuteToPosition(chute, MechanismPosition.INTAKE)
                ).andThen(
                        EndEffectorComponents.intakeCoral(endEffector)
                )).andThen(
                new ChuteToPosition(chute, MechanismPosition.REST)
        );
    }

    public static Command intakeAlgae(EndEffector endEffector) {
        return 
            EndEffectorComponents.intakeAlgae(endEffector)
            .until(endEffector::algaeSensorTripped)
            .andThen(Commands.waitSeconds(1))
            .andThen(new InstantCommand(() -> endEffector.setWheelDutyCycle(0.1)));
    }
}