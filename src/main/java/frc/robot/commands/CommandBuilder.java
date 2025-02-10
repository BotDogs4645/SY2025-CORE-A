package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Constants.VisionConstants;
import frc.robot.lib.util.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class CommandBuilder {
  // Test commit
  public class Vision {

    public double getDistanceToAprilTag(String limelightName) {
      // Get the target's 3D pose in robot space
      Pose3d tagPose = LimelightHelpers.getTargetPose3d_RobotSpace(limelightName);

      // Extract the x, y, and z distances
      double x = tagPose.getX();
      double y = tagPose.getY();

      // Calculate the horizontal distance (2D, ignoring height)
      double distance2D = Math.sqrt(x * x + y * y);

      // Return the desired distance (e.g., horizontal distance)
      return distance2D;
    }

    public Command driveDistance(double distanceMeters, CommandSwerveDrivetrain drivetrain) {
      PIDController distanceController = new PIDController(1.0, 0, 0); // Adjust gains as needed
      distanceController.setSetpoint(0); // Drive until the error is 0
      distanceController.setTolerance(0.05); // Allowable error in meters

      return Commands.run(
              () -> {
                // Get the current pose of the robot
                Pose2d currentPose = drivetrain.getState().Pose;

                // Calculate the error (remaining distance to drive)
                double error = distanceMeters - currentPose.getTranslation().getNorm();

                // Calculate the speed using the PID controller
                double speed = distanceController.calculate(error);

                // Drive the robot forward at the calculated speed
                drivetrain.applyRequest(
                    () ->
                        new SwerveRequest.FieldCentric()
                            .withVelocityX(speed) // Move forward
                            .withVelocityY(0) // No sideways movement
                            .withRotationalRate(0) // No rotation
                    );
              },
              drivetrain)
          .until(() -> distanceController.atSetpoint()) // Stop when the error is within tolerance
          .finallyDo(
              (interrupted) -> {
                // Stop the drivetrain when the command ends
                drivetrain.applyRequest(
                    () ->
                        new SwerveRequest.FieldCentric()
                            .withVelocityX(0)
                            .withVelocityY(0)
                            .withRotationalRate(0));

                distanceController.close();
              });
    }

    public Command driveToTag(String limelightName, CommandSwerveDrivetrain drivetrain) {
      return Commands.sequence(
          // Step 1: Face the target
          faceTarget(limelightName, drivetrain),

          // Step 2: Calculate the distance to the target and drive to it
          Commands.runOnce(
              () -> {
                double distanceToTag = getDistanceToAprilTag(limelightName);

                double adjustedDistance = distanceToTag - VisionConstants.distanceOffset;
                // Drive to the calculated distance
                driveDistance(adjustedDistance, drivetrain).schedule();
              }));
    }

    public static Command aimAtTarget(
        String limelightName, CommandSwerveDrivetrain swerveSubsystem) {
      // Access the Limelight's network table
      NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable(limelightName);

      // Create a PID controller for rotation (adjust gains as needed)
      PIDController rotationController = new PIDController(0.05, 0, 0); // Example gains

      // Define the command
      return new FunctionalCommand(
              // Initialize: Reset the PID controller
              () -> {
                rotationController.reset();
                rotationController.setSetpoint(0); // Aim for tx = 0 (centered on target)
                rotationController.setTolerance(1); // Allowable error in degrees
              },
              // Execute: Read Limelight data and drive the swerve drivetrain
              () -> {
                double tx = limelightTable.getEntry("tx").getDouble(0); // Horizontal offset
                double tv = limelightTable.getEntry("tv").getDouble(0); // Target detected

                if (tv == 1) {
                  // Calculate rotation speed using the PID controller
                  double rotationSpeed = rotationController.calculate(tx);

                  // Drive the swerve drivetrain (rotate only, no translation)
                  swerveSubsystem.applyRequest(
                      () ->
                          new SwerveRequest.FieldCentric()
                              .withVelocityX(0) // No X movement
                              .withVelocityY(0) // No Y movement
                              .withRotationalRate(rotationSpeed) // Rotate to align with target
                      );
                } else {
                  // Stop the drivetrain if no target is detected
                  swerveSubsystem.applyRequest(
                      () ->
                          new SwerveRequest.FieldCentric()
                              .withVelocityX(0)
                              .withVelocityY(0)
                              .withRotationalRate(0));
                }
              },
              // End: Stop the drivetrain
              (interrupted) -> {
                swerveSubsystem.applyRequest(
                    () ->
                        new SwerveRequest.FieldCentric()
                            .withVelocityX(0)
                            .withVelocityY(0)
                            .withRotationalRate(0));
                rotationController.close();
              },
              // IsFinished: End the command when the target is aligned
              () -> rotationController.atSetpoint(),
              // Require the swerve subsystem
              swerveSubsystem)
          .withTimeout(5); // Add a timeout to prevent the command from running indefinitel
    }

    public static Command faceTarget(
        String limelightName, CommandSwerveDrivetrain swerveSubsystem) {
      // Access the Limelight's network table
      NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable(limelightName);

      // Create a PID controller for rotation (adjust gains as needed)
      PIDController rotationController = new PIDController(0.05, 0, 0); // Example gains

      // Define the command
      return new FunctionalCommand(
              // Initialize: Reset the PID controller
              () -> {
                rotationController.reset();
                rotationController.setSetpoint(0); // Aim for tx = 0 (centered on target)
                rotationController.setTolerance(1); // Allowable error in degrees
              },
              // Execute: Read Limelight data and drive the swerve drivetrain
              () -> {
                double tx = limelightTable.getEntry("tx").getDouble(0); // Horizontal offset
                double tv = limelightTable.getEntry("tv").getDouble(0); // Target detected

                if (tv == 1) {
                  // Calculate rotation speed using the PID controller
                  double rotationSpeed = rotationController.calculate(tx);

                  // Drive the swerve drivetrain (rotate only, no translation)
                  swerveSubsystem.applyRequest(
                      () ->
                          new SwerveRequest.FieldCentric()
                              .withVelocityX(0) // No X movement
                              .withVelocityY(0) // No Y movement
                              .withRotationalRate(rotationSpeed) // Rotate to align with target
                      );
                } else {
                  // Stop the drivetrain if no target is detected
                  swerveSubsystem.applyRequest(
                      () ->
                          new SwerveRequest.FieldCentric()
                              .withVelocityX(0)
                              .withVelocityY(0)
                              .withRotationalRate(0));
                }
              },
              // End: Stop the drivetrain
              (interrupted) -> {
                swerveSubsystem.applyRequest(
                    () ->
                        new SwerveRequest.FieldCentric()
                            .withVelocityX(0)
                            .withVelocityY(0)
                            .withRotationalRate(0));
              },
              // IsFinished: End the command when the target is aligned
              () -> rotationController.atSetpoint(),
              // Require the swerve subsystem
              swerveSubsystem)
          .withTimeout(5); // Add a timeout to prevent the command from running indefinitely
    }
  }
}
