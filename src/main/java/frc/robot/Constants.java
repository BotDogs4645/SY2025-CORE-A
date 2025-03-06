package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class Constants {

    public static class ClimberConstants {
      public static int climbMotorAID = 19;
      public static int climbMotorBID = 20;
      public static int funnelMotorID = 21;

      public static int encoderDioPort = 1;

      public static double climbGearRatio = 100; // Gearbox reduction (x:1)
      public static double funnelGearRatio = 50; // Gearbox reduction (x:1)

      public static Rotation2d deployPosition = Rotation2d.fromDegrees(0);
      public static Rotation2d climbPosition = Rotation2d.fromDegrees(0);

      public static Rotation2d rotationThreshold = Rotation2d.fromRotations(0.02);
      public static AngularVelocity velocityThreshold = RadiansPerSecond.of(0.02);
      
      public static class FunnelConstants {
        public static double KP = 20;
        public static double KI = 0;
        public static double KD = 0;
      }
    }

    public static class EndEffectorConstants {
      public static int manipulateMotorID = 17;
      public static int pivotMotorID = 18;
      public static int firstCoralSensorID = 22;
      public static int secondCoralSensorID = 23;
      public static int algaeSensorID = 24;

      public static int encoderDioPort = 0;

      public static Rotation2d rotationThreshold = Rotation2d.fromRotations(0.02);
      public static AngularVelocity velocityThreshold = RadiansPerSecond.of(0.02);

      public static double KP = 50;
      public static double KI = 0;
      public static double KD = 0;

      public static double gearRatio = 36;
      
      public static Distance coralThreshold = Millimeters.of(50);
      public static Distance algaeThreshold = Millimeters.of(50);
    }

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

        // max: 1.36m

        public static int leftMotorCANId = 15; //CANivore bus
        public static int rightMotorCANId = 16; //CANivore bus
        public static double UpSpeed = 0.6;
        public static double DownSpeed = -1;

        private static double gearRatio = 49; // Gearbox reduction (x:1)
        private static Distance linearConversion = Inches.of(22 * 0.25); // 22t sprocket * 0.25" pitch
        private static int numStages = 2;
        public static double metersPerRotation = (numStages * linearConversion.in(Meters)) / gearRatio;

        public static double KP = 0.5;
        public static double KI = 0;
        public static double KD = 0;
        
        public static Distance positionThreshold = Meters.of(0.05);
        public static LinearVelocity velocityThreshold = MetersPerSecond.of(0.02);
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

    public enum MechanismPosition {
      SCORE_L1(0.48, 0.3),
      SCORE_L2(0.47616563690414093, 0.3802631995376275),
      SCORE_L3(0.47616563690414093, 0.8088684570312499),
      SCORE_L4(0.46149188653729717, 1.4074546695631376),
      DEALGAE_LOW(0.48, 0.3),
      DEALGAE_HIGH(0.48, 0.6),
      PROCESSOR(0.48, 0.3),
      INTAKE(0.41, 0),
      REST(0.48, 0),
      DEPLOY(0.48, 0),
      CLIMB(0.48, 0);

      public final Rotation2d pivotPosition;
      public final Distance elevatorPosition;

      private MechanismPosition(double pivotPosition, double elevatorPosition) {
        this.pivotPosition = Rotation2d.fromRotations(pivotPosition);
        this.elevatorPosition = Meters.of(elevatorPosition);
      }

    }
}