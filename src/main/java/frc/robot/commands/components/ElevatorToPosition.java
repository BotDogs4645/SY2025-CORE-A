package frc.robot.commands.components;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class ElevatorToPosition extends Command {

  private Elevator elevator;
  PositionDutyCycle control;

  public ElevatorToPosition(Elevator elevator, double position) {
    this.elevator = elevator;
    control = new PositionDutyCycle(position);
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    elevator.setControl(control);

  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    elevator.setBrake();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}