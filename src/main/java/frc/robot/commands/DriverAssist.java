package frc.robot.commands;

import java.util.List;
import java.util.Set;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriverAssist {

    public static Command generatePathfindCommandPathFinder(CommandSwerveDrivetrain drivetrain, Pose2d endpoint) {

        Pose2d curPose = drivetrain.getState().Pose;

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                curPose,
                endpoint);

        PathConstraints constraints = new PathConstraints(1, 1, 2 * Math.PI, 4 * Math.PI);
        PathPlannerPath alignmentPath = new PathPlannerPath(
                waypoints,
                constraints,
                null,
                new GoalEndState(0, endpoint.getRotation()));
        Logger.recordOutput("Vision/driverAssist/curPose", curPose);
        Logger.recordOutput("Vision/driverAssist/goalPose", endpoint);
        return AutoBuilder.followPath(alignmentPath);
    }

    public static Command reefPathfindCommand(CommandSwerveDrivetrain drivetrain, boolean isLeft, boolean isBlue) {
        Pose2d curPose = drivetrain.getState().Pose;
        final Pose2d goalPose = isBlue
                ? (isLeft ? curPose.nearest(Constants.ReefPoses.blueLeftReefPoses)
                          : curPose.nearest(Constants.ReefPoses.blueRightReefPoses))
                : (isLeft ? curPose.nearest(Constants.ReefPoses.redLeftReefPoses)
                          : curPose.nearest(Constants.ReefPoses.redRightReefPoses));

        Logger.recordOutput("Vision/driverAssist/poses", Constants.ReefPoses.blueLeftReefPoses.toArray(new Pose2d[0]));

        return new DeferredCommand(() -> generatePathfindCommandPathFinder(drivetrain, goalPose), Set.of(drivetrain));
    }
}
