package frc.robot;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

public class Telemetry {

    /** Accept the swerve drive state and telemeterize it to AdvantageKit. */
    public static void telemeterize(SwerveDriveState state) {
        /* Telemeterize the swerve drive state */
        Logger.recordOutput("Drive/Pose", state.Pose);
        Logger.recordOutput("Drive/Speeds", state.Speeds);

        Logger.recordOutput("Drive/OdometryPeriod", state.OdometryPeriod);

        Logger.recordOutput("Drive/ModuleStates", state.ModuleStates);
        Logger.recordOutput("Drive/ModuleTargets", state.ModuleTargets);
    }
}
