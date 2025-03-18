package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.MechanismPosition;
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
            .andThen(new ChuteToPosition(chute, MechanismPosition.REST));
    }

    public static Command toMechanismPosition(EndEffector endEffector, Elevator elevator, MechanismPosition position) {
        return new ParallelCommandGroup(
            new ElevatorToPosition(elevator, position),
            new EndEffectorToPosition(endEffector, position)
        );
    }

    public static Command intakeSequence(Chute chute, EndEffector endEffector, Elevator elevator) {
        return toMechanismPosition(endEffector, elevator, MechanismPosition.INTAKE)
        .andThen(
            new ChuteToPosition(chute, MechanismPosition.INTAKE)
        ).andThen(
            EndEffectorComponents.intakeCoral(endEffector)
        ).andThen(
            new ChuteToPosition(chute, MechanismPosition.REST)
        );
    }

    public static Command intakeAlgaeLow(EndEffector endEffector, Elevator elevator) {
        return new ParallelCommandGroup(
            new ElevatorToPosition(elevator, MechanismPosition.DEALGAE_LOW),
            new EndEffectorToPosition(endEffector, MechanismPosition.DEALGAE_LOW)
        ).andThen(
            EndEffectorComponents.intakeAlgae(endEffector)
            .until(() -> endEffector.algaeSensorTripped())
            .andThen(Commands.waitSeconds(1))
            .andThen(new InstantCommand(() -> endEffector.setWheelDutyCycle(0.05)))
        );
    }
    

    public static Command intakeAlgaeHigh(EndEffector endEffector, Elevator elevator) {
        return new ParallelCommandGroup(
            new ElevatorToPosition(elevator, MechanismPosition.DEALGAE_HIGH),
            new EndEffectorToPosition(endEffector, MechanismPosition.DEALGAE_HIGH)
        ).andThen(
            EndEffectorComponents.intakeAlgae(endEffector)
        ).andThen(
            toMechanismPosition(endEffector, elevator, MechanismPosition.REST)
        );
    }

    public static Command score(EndEffector endEffector, Elevator elevator) {
        return EndEffectorComponents.score(endEffector)
        .andThen(new WaitCommand(0.5))
        .andThen(
            toMechanismPosition(endEffector, elevator, MechanismPosition.REST)
        );
    }
}