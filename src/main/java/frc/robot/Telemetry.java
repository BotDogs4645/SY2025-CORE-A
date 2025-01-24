package frc.robot;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;

public class Telemetry {

    /** Accept the swerve drive state and telemeterize it to AdvantageKit. */
    public static void telemeterizeSwerve(SwerveDriveState state) {
        /* Telemeterize the swerve drive state */
        Logger.recordOutput("Drive/Pose", state.Pose);
        Logger.recordOutput("Drive/Speeds", state.Speeds);

        Logger.recordOutput("Drive/OdometryPeriod", state.OdometryPeriod);

        Logger.recordOutput("Drive/ModuleStates", state.ModuleStates);
        Logger.recordOutput("Drive/ModuleTargets", state.ModuleTargets);
    }

    public static void telemeterizeTrajectory(Trajectory<SwerveSample> trajectory, boolean starting) {
        Logger.recordOutput("Choreo/Trajectory", trajectory.getPoses());
        Logger.recordOutput("Choreo/Duration", trajectory.getTotalTime());
        Logger.recordOutput("Choreo/Running", starting);
    }
}
