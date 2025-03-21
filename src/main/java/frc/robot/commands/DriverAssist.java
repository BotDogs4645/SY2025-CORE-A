package frc.robot.commands;

import java.lang.reflect.Array;
import java.util.List;
import java.util.Set;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.Limelight;

public class DriverAssist {

    public static Command generatePathfindCommandPathFinder(CommandSwerveDrivetrain drivetrain, Pose2d endpoint) {

        Pose2d curPose = drivetrain.getState().Pose;
        Pose2d goalPose = endpoint;

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                curPose,
                goalPose);

        PathConstraints constraints = new PathConstraints(1, 1, 2 * Math.PI, 4 * Math.PI);
        PathPlannerPath alignmentPath = new PathPlannerPath(
                waypoints,
                constraints,
                null,
                new GoalEndState(0, goalPose.getRotation()));
        Logger.recordOutput("Vision/driverAssist/curPose", curPose);
        Logger.recordOutput("Vision/driverAssist/goalPose", goalPose);
        return AutoBuilder.followPath(alignmentPath);
    }

    public static Command reefPathfindCommand(CommandSwerveDrivetrain drivetrain, boolean isLeft, boolean isBlue) {
        // return generatePathfindCommand(drivetrain, new Pose2d(3.38, 4.09, new
        // Rotation2d(0)));

        Pose2d curPose = drivetrain.getState().Pose;
        final Pose2d goalPose = isBlue
                ? (isLeft ? curPose.nearest(Constants.ReefPoses.blueLeftReefPoses)
                          : curPose.nearest(Constants.ReefPoses.blueRightReefPoses))
                : (isLeft ? curPose.nearest(Constants.ReefPoses.redLeftReefPoses)
                          : curPose.nearest(Constants.ReefPoses.redRightReefPoses));

        return new DeferredCommand(() -> generatePathfindCommandPathFinder(drivetrain, goalPose), Set.of(drivetrain));
    }
}
