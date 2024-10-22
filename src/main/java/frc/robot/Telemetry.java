package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;

public class Telemetry {
    /**
     * Construct a telemetry object.
     * 
     */
    public Telemetry() {}

    /* What to publish over networktables for telemetry */
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    /* Robot pose for field positioning */
    private final NetworkTable table = inst.getTable("Pose");
    private final DoubleArrayPublisher fieldPub = table.getDoubleArrayTopic("robotPose").publish();
    private final StringPublisher fieldTypePub = table.getStringTopic(".type").publish();

    /* Robot speeds for general checking */
    private final NetworkTable driveStats = inst.getTable("Drive");
    private final DoublePublisher velocityX = driveStats.getDoubleTopic("Velocity X").publish();
    private final DoublePublisher velocityY = driveStats.getDoubleTopic("Velocity Y").publish();
    private final DoublePublisher speed = driveStats.getDoubleTopic("Speed").publish();
    private final DoublePublisher odomPeriod = driveStats.getDoubleTopic("Odometry Period").publish();

    /* Measured and Desired module states */
    StructArrayPublisher<SwerveModuleState> moduleStates = NetworkTableInstance.getDefault()
        .getStructArrayTopic("AdvantageScope/ModuleStates", SwerveModuleState.struct).publish();
    StructArrayPublisher<SwerveModuleState> desiredStates = NetworkTableInstance.getDefault()
        .getStructArrayTopic("AdvantageScope/DesiredStates", SwerveModuleState.struct).publish();

    /* Keep a reference of the last pose to calculate the speeds */
    private Pose2d m_lastPose = new Pose2d();
    private double lastTime = Utils.getCurrentTimeSeconds();

    /* Accept the swerve drive state and telemeterize it to smartdashboard */
    public void telemeterize(SwerveDriveState state) {
        /* Telemeterize the pose */
        Pose2d pose = state.Pose;
        fieldTypePub.set("Field2d");
        fieldPub.set(new double[] {
            pose.getX(),
            pose.getY(),
            pose.getRotation().getRadians()
        });

        /* Telemeterize the robot's general speeds */
        double currentTime = Utils.getCurrentTimeSeconds();
        double diffTime = currentTime - lastTime;
        lastTime = currentTime;
        Translation2d distanceDiff = pose.minus(m_lastPose).getTranslation();
        m_lastPose = pose;

        Translation2d velocities = distanceDiff.div(diffTime);

        speed.set(velocities.getNorm());
        velocityX.set(velocities.getX());
        velocityY.set(velocities.getY());
        odomPeriod.set(state.OdometryPeriod);

        /* Telemeterize the module's states */
        moduleStates.set(state.ModuleStates);
        desiredStates.set(state.ModuleTargets);
    }
}
