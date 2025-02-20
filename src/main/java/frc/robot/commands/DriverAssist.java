package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class DriverAssist {
    public static Command generatePathfindCommand(Subsystem drivetrain, Pose2d endpoint) {
        return drivetrain.defer(
            () -> AutoBuilder.pathfindToPose(endpoint, Constants.PathPlannerConstants.pathConstraints)
        );
    }

    public static Command reefPathfindCommand(Subsystem drivetrain) {
        return generatePathfindCommand(drivetrain, new Pose2d(0, 0, new Rotation2d(0)));
    }
}