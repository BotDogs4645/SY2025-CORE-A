package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.components.ElevatorDown;
import frc.robot.subsystems.Elevator;

public class CommandBuilder {

    public static Command HomeElevator(Elevator elevator) {
        return new ElevatorDown(elevator)
                .until(elevator::getLimitSwitch)
                .andThen(() -> {elevator.resetEncoders();}, elevator);
    }

    public static Command ElevatorToHeight(Elevator elevator, double height) {
        return new Command() {
            
        };
    }

}
