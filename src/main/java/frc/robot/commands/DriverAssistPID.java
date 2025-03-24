package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

class PIDControllerConfigurable extends PIDController {
    public PIDControllerConfigurable(double kP, double kI, double kD) {
        super(kP, kI, kD);
    }

    public PIDControllerConfigurable(double kP, double kI, double kD, double tolerance) {
        super(kP, kI, kD);
        this.setTolerance(tolerance);
    }
}

public class DriverAssistPID extends Command {
    private static CommandSwerveDrivetrain drivetrain;
    private static Pose2d endpoint;
    private static final PIDControllerConfigurable rotationalPidController = new PIDControllerConfigurable(0.05000,
            0.000000, 0.001000, 0.01);
    private static final PIDControllerConfigurable xPidController = new PIDControllerConfigurable(0.400000,
            0.000000, 0.000600, 0.01);
    private static final PIDControllerConfigurable yPidController = new PIDControllerConfigurable(0.3, 0, 0, 0.3);
    private static final SwerveRequest.RobotCentric alignRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private static final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

    public double rotationalRate = 0;
    public double velocityX = 0;

    public DriverAssistPID(CommandSwerveDrivetrain drivetrain, Pose2d endpoint) {
        DriverAssistPID.drivetrain = drivetrain;
        DriverAssistPID.endpoint = endpoint;

    }


    @Override
    public void execute() {

        Pose2d curPose = drivetrain.getState().Pose;

        double txnc = endpoint.getRotation().getDegrees() - curPose.getRotation().getDegrees();
        rotationalRate = rotationalPidController.calculate(2 * txnc, 0.0) * 0.75 * 0.9;

        double distToRobot = curPose.getTranslation().getDistance(endpoint.getTranslation());

        double velocityX = xPidController.calculate(distToRobot, 0.1)
                * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.7;

        if (rotationalPidController.atSetpoint() && xPidController.atSetpoint()) {
            this.end(true);
        }

        Logger.recordOutput("DriverAssist/curPose", curPose);
        Logger.recordOutput("DriverAssist/txnc", txnc);
        Logger.recordOutput("DriverAssist/rotationalRate", rotationalRate);
        Logger.recordOutput("DriverAssist/distanceToRobot", distToRobot);
        Logger.recordOutput("DriverAssist/velocityX", velocityX);
        
    }

    @Override
    public boolean isFinished() {
      return rotationalPidController.atSetpoint() && xPidController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.applyRequest(() -> idleRequest);
    }


}
