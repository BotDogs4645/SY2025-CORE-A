package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;

public class Constants {
    public class ChoreoConstants {
        public static PIDController xController = new PIDController(10.0, 0.0, 0.0);
        public static PIDController yController = new PIDController(10.0, 0.0, 0.0);
        public static PIDController headingController = new PIDController(7.5, 0.0, 0.0);
    }

    public class VisionConstants {
        public static String[] limelightNames = {"limelight"};

        // Basic filtering thresholds
        public static double maxAmbiguity = 0.3;
        public static double maxZError = 0.75;

        // Ignore MT2 readings when robot is rotationg too fast
        public static double maxAngluarVelocity = 2 * Math.PI; // Rad/s

        public static AprilTagFieldLayout aprilTagLayout = 
            AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

        // Standard deviation baselines, for 1 meter distance and 1 tag
        // (Adjusted automatically based on distance and # of tags)
        public static double linearStdDevBaseline = 0.02; // Meters
        public static double angularStdDevBaseline = 0.06; // Radians

        // Multipliers to apply for MegaTag 2 observations
        public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
        public static double angularStdDevMegatag2Factor = 
            Double.POSITIVE_INFINITY; // No rotation data available

        // Standard deviation multipliers for each camera
        // (Adjust to trust some cameras more than others)
        public static double[] limelightStdDevFactors = {1.0};
    }

    public class EndEffectorConstants {
        public static final int pivotGearRatio = 10; // wrong prolly
        public static final Rotation2d level1Degrees = Rotation2d.fromDegrees(0);
        public static final Rotation2d level2Degrees = Rotation2d.fromDegrees(0);
        public static final Rotation2d level3Degrees = Rotation2d.fromDegrees(0);
        public static final Rotation2d level4Degrees = Rotation2d.fromDegrees(0);

    }

    public class MotionMagicConstants {
        public static final double kS = 0.25;
        public static final double kV = 0.12;
        public static final double kA = 0.01;
        public static final double kP = 4.8;
        public static final double kI = 0.0;
        public static final double kD = 0.1;

    }
}