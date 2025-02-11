package frc.robot.commands;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.components.ElevatorDown;
import frc.robot.subsystems.Elevator;

public class CommandBuilder {

    public static Command HomeElevator(Elevator elevator) {
        return new ElevatorDown(elevator)
                .until(elevator::getLimitSwitch)
                .andThen(() -> {
                    elevator.enableBrakemode();
                    elevator.resetEncoders();
                }, elevator);
    }
    // public static Command CoastElevator(Elevator elevator) {
    //     return Commands.runOnce(() -> {
    //         elevator.enableCoastMode();
    //     }, null);
    // }

    public static Command ElevatorToHeight(Elevator elevator, double height) {
        return new InstantCommand(() -> {
            final PositionDutyCycle m_request = new PositionDutyCycle(5).withSlot(0);
            elevator.setControl(m_request.withPosition(5));
        });
    }

}
