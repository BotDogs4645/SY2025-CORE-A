package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class Constants {

    public static class PathPlannerConstants {
        public static RobotConfig config;
    
        static {
          try{
            PathPlannerConstants.config = RobotConfig.fromGUISettings();
          } catch (Exception e) {
            e.printStackTrace();
          }
        }
    
        public static final PIDConstants translationPID = new PIDConstants(5, 0, 0);
        public static final PIDConstants rotationPID = new PIDConstants(5, 0, 0);
    
        // Speed and acceleration limits for on the fly path generation
        public static final PathConstraints pathConstraints = new PathConstraints(
          MetersPerSecond.of(5),
          MetersPerSecondPerSecond.of(10),
          RotationsPerSecond.of(0.75),
          RotationsPerSecondPerSecond.of(5)
        );
      }    

    public class ElevatorConstants {
        public static int leftMotorCANId = 15; //CANivore bus
        public static int rightMotorCANId = 16; //CANivore bus
        public static double UpSpeed = 0.6;
        public static double DownSpeed = -1;

        private static double gearRatio = 10; // Gearbox reduction (x:1)
        private static Distance linearConversion = Inches.of(22 * 0.25); // 22t sprocket * 0.25" pitch
        private static int numStages = 2;
        public static double metersPerRotation = (numStages * linearConversion.in(Meters)) / gearRatio;

        public static double KP = 0.02;
        public static double KI = 0;
        public static double KD = 0;
        
        public static Distance positionThreshold = Meters.of(0.05);
        public static LinearVelocity velocityThreshold = MetersPerSecond.of(0.02);
        public class Heights {
            public static Distance level1 = Meters.of(0.3);
            public static Distance level2 = Meters.of(0.6);
            public static Distance level3 = Meters.of(0.9);
            public static Distance level4 = Meters.of(1.5);
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