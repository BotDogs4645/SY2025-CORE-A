package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;

public class Constants {
    public class ChoreoConstants {
        public static PIDController xController = new PIDController(10.0, 0.0, 0.0);
        public static PIDController yController = new PIDController(10.0, 0.0, 0.0);
        public static PIDController headingController = new PIDController(7.5, 0.0, 0.0);
    }

    public class ElevatorConstants {
        public static int leftMotorCANId = 14;
        public static int rightMotorCANId = 3;
        public static int lowerLimitSwitchDIOPort = 0;
        public static double elevatorSpeed = 0.05;
        public static double targetThreshold = 0.6;

        public class Heights {
            public static double level1 = 0.3;
            public static double level2 = 0.6;
            public static double level3 = 0.9;
            public static double level4 = 1.5;
        }
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
}