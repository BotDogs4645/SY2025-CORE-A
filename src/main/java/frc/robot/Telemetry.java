package frc.robot;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Telemetry {
    /**
     * Construct a telemetry object.
     * 
     */
    public Telemetry() {}

    /* Keep a reference of the last pose to calculate the speeds */
    private Pose2d m_lastPose = new Pose2d();
    private double lastTime = Utils.getCurrentTimeSeconds();

    /* Accept the swerve drive state and telemeterize it to smartdashboard */
    public void telemeterize(SwerveDriveState state) {
        /* Telemeterize the pose */
        Pose2d pose = state.Pose;
        Logger.recordOutput("Drive/Pose", pose);

        /* Telemeterize the robot's general speeds */
        double currentTime = Utils.getCurrentTimeSeconds();
        double diffTime = currentTime - lastTime;
        lastTime = currentTime;
        Translation2d distanceDiff = pose.minus(m_lastPose).getTranslation();
        m_lastPose = pose;

        Translation2d velocities = distanceDiff.div(diffTime);

        Logger.recordOutput("Drive/Velocities", velocities);
        Logger.recordOutput("Drive/OdometryPeriod", state.OdometryPeriod);

        /* Telemeterize the module's states */
        Logger.recordOutput("Drive/ModuleStates", state.ModuleStates);
        Logger.recordOutput("Drive/DesiredStates", state.ModuleTargets);
    }
}
