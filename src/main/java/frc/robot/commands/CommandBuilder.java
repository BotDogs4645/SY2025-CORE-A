package frc.robot.commands;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.commands.components.ElevatorDown;
import frc.robot.commands.components.ElevatorToPosition;
import frc.robot.subsystems.Elevator;

public class CommandBuilder {

    public static Command HomeElevator(Elevator elevator) {
        return new ElevatorDown(elevator)
                .until(elevator::getLimitSwitch)
                .andThen(() -> {
                    elevator.setBrake();
                    elevator.resetEncoders();
                }, elevator);
    }
    // public static Command CoastElevator(Elevator elevator) {
    // return Commands.runOnce(() -> {
    // elevator.enableCoastMode();
    // }, null);
    // }

    public static Command ElevatorToLevel(Elevator elevator, int level) {
        double position = 0;
        switch (level) {
            case 1:
                position = Constants.ElevatorConstants.Heights.level1;
                break;
            case 2:
                position = Constants.ElevatorConstants.Heights.level2;
                break;
            case 3:
                position = Constants.ElevatorConstants.Heights.level3;
                break;
            case 4:
                position = Constants.ElevatorConstants.Heights.level4;
                break;
            default:
                return Commands.none();
        }
        return new ElevatorToPosition(elevator, position)
                .until(elevator::hasReachedTarget);
    }

}
