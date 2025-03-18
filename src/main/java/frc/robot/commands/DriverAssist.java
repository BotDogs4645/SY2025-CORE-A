package frc.robot.commands;

import java.util.List;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriverAssist {
    public static Command generatePathfindCommand(CommandSwerveDrivetrain drivetrain, Pose2d endpoint) {
        // try {
        // PathPlannerPath path = PathPlannerPath.fromPathFile("Reef 18");
        // return drivetrain.defer(
        // () -> AutoBuilder.pathfindThenFollowPath(path,
        // Constants.PathPlannerConstants.pathConstraints));
        // } catch (Exception e) {
        // return new Command() {
        // };
        // }
        // return drivetrain.defer(
        // () -> AutoBuilder.pathfindToPose(endpoint,
        // Constants.PathPlannerConstants.pathConstraints)
        // );

        // return Commands.runOnce(() -> {
        // try {
        // PathPlannerPath path = PathPlannerPath.fromPathFile("Reef 18");
        // drivetrain.defer(() -> AutoBuilder.pathfindThenFollowPath(path,
        // Constants.PathPlannerConstants.pathConstraints));
        // } catch (Exception e) {
        // e.printStackTrace();
        // }
        // }, drivetrain);
        

        Pose2d curPose = drivetrain.getState().Pose;
        Pose2d goalPose = endpoint;

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(curPose.getX(), curPose.getY(), Rotation2d.fromDegrees(0)),
                new Pose2d(goalPose.getX(), goalPose.getY(), Rotation2d.fromDegrees(0)));

        PathConstraints constraints = new PathConstraints(1, 1, 2 * Math.PI, 4 * Math.PI);
        PathPlannerPath alignmentPath = new PathPlannerPath(
                waypoints,
                constraints,
                null,
                new GoalEndState(0, goalPose.getRotation()));
        return AutoBuilder.followPath(alignmentPath);
    }

    public static Command reefPathfindCommand(CommandSwerveDrivetrain drivetrain) {
        // return generatePathfindCommand(drivetrain, new Pose2d(3.38, 4.09, new
        // Rotation2d(0)));
        return new DeferredCommand(() -> generatePathfindCommand(drivetrain, new Pose2d(2.945,4.103, new Rotation2d(0))), Set.of(drivetrain));
    }
}
