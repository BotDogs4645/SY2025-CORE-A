import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class ElevatorDown extends Command {

    private Elevator elevator;

    public ElevatorDown(Elevator elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        elevator.setSpeed(Constants.ElevatorConstants.elevatorSpeed);
    }

    @Override
    public void end(boolean interrupted) {
      elevator.stop();
    }

    

    @Override
    public boolean isFinished() {
      return false;
    }



}