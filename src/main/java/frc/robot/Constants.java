package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import java.util.List;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class Constants {

    public static class ClimberConstants {
      public static int motorID = 19;

      public static int encoderDioPort = 4;

      public static double gearRatio = 100 * (70.0 / 30.0); // Gearbox reduction (x:1)
    }

    public static class ChuteConstants {
      public static int motorID = 21;

      public static int encoderDioPort = 2;

      public static double gearRatio = 48 * (60.0 / 24.0); // Gearbox reduction (x:1)

      public static Rotation2d intakePosition = Rotation2d.fromRotations(0.10);
      public static Rotation2d deployPosition = Rotation2d.fromRotations(0.20);
      public static Rotation2d restPosition = Rotation2d.fromRotations(0.18);
      public static Rotation2d climbPosition = Rotation2d.fromRotations(0.42);
      public static Rotation2d stowPosition = Rotation2d.fromRotations(0.14672542491813567);

      public static Rotation2d rotationThreshold = Rotation2d.fromRotations(0.02);
      public static AngularVelocity velocityThreshold = RadiansPerSecond.of(0.02);

      public static double KP = 30;
      public static double KI = 0;
      public static double KD = 0;
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
      public static double KI = 1;
      public static double KD = 0;
      public static double KG = 0;

      public static double gearRatio = 36;
      
      public static Distance coralThreshold = Millimeters.of(50);
      public static Distance algaeThreshold = Millimeters.of(30);

      public static Rotation2d safetyAngle = Rotation2d.fromRotations(0.39);
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
    
        public static final PIDConstants translationPID = new PIDConstants(1, 1, 0);
        public static final PIDConstants rotationPID = new PIDConstants(5, 0, 0);
    
        // Speed and acceleration limits for on the fly path generation
        public static final PathConstraints pathConstraints = new PathConstraints(
          MetersPerSecond.of(1),
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
        public static double maxAmbiguity = 0.2;
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
      SCORE_L1(0.48325876208146906, 0.1),
      SCORE_L2(0.48325876208146906, 0.3802631995376275),
      SCORE_L3(0.48325876208146906, 0.8088684570312499),
      SCORE_L4(0.48325876208146906, 1.4074546695631376),
      DEALGAE_LOW(0.44520938613023464, 0.6145452447385203),
      DEALGAE_HIGH(0.44520938613023464, 1.001),
      DEALGAE_GROUND(0.49749486243737157, 0),
      PROCESSOR(0.48, 0.3),
      INTAKE(0.44485016112125403, 0),
      REST(0.40, 0),
      DEPLOY(0.48, 0),
      CLIMB(0.48, 0),
      STOW(0.22436090560902264, 0);

      public final Rotation2d pivotPosition;
      public final Distance elevatorPosition;

      private MechanismPosition(double pivotPosition, double elevatorPosition) {
        this.pivotPosition = Rotation2d.fromRotations(pivotPosition);
        this.elevatorPosition = Meters.of(elevatorPosition);
      }
      
    }

    public class ReefPoses {
        public static final List<Pose2d> blueLeftReefPoses = List.of(
            new Pose2d(2.765, 4.190, new Rotation2d(0)),    // 18-2
            new Pose2d(3.748, 5.590, new Rotation2d(Math.toRadians(-60))), // 19-2
            new Pose2d(5.500, 5.430, new Rotation2d(Math.toRadians(-120))), // 20-2
            new Pose2d(6.200, 3.857, new Rotation2d(Math.toRadians(180))),  // 21-2
            new Pose2d(5.200, 2.446, new Rotation2d(Math.toRadians(120))),  // 22-2
            new Pose2d(3.517, 2.573, new Rotation2d(Math.toRadians(60)))    // 17-2
        );

        public static final List<Pose2d> blueRightReefPoses = List.of(
            new Pose2d(2.765, 3.850, new Rotation2d(0)),    // 18-1
            new Pose2d(3.476, 5.428, new Rotation2d(Math.toRadians(-60))), // 19-1
            new Pose2d(5.210, 5.600, new Rotation2d(Math.toRadians(-120))), // 20-1
            new Pose2d(6.211, 4.168, new Rotation2d(Math.toRadians(180))),  // 21-1
            new Pose2d(5.497, 2.629, new Rotation2d(Math.toRadians(120))),  // 22-1
            new Pose2d(3.765, 2.454, new Rotation2d(Math.toRadians(60)))    // 17-1
        );

        public static final List<Pose2d> redLeftReefPoses = List.of(
            new Pose2d(11.336, 4.204, new Rotation2d(0)),    // 10-2
            new Pose2d(12.362, 5.597, new Rotation2d(Math.toRadians(-60))), // 9-2
            new Pose2d(14.084, 5.421, new Rotation2d(Math.toRadians(-120))), // 8-2
            new Pose2d(14.783, 3.851, new Rotation2d(Math.toRadians(180))),  // 7-2
            new Pose2d(13.792, 2.461, new Rotation2d(Math.toRadians(120))),  // 6-2
            new Pose2d(12.092, 2.599, new Rotation2d(Math.toRadians(60)))    // 11-2
        );

        public static final List<Pose2d> redRightReefPoses = List.of(
            new Pose2d(11.348, 3.863, new Rotation2d(0)),    // 10-1
            new Pose2d(12.075, 5.430, new Rotation2d(Math.toRadians(-60))), // 9-1
            new Pose2d(13.790, 5.687, new Rotation2d(Math.toRadians(-120))), // 8-1
            new Pose2d(14.778, 5.171, new Rotation2d(Math.toRadians(180))),  // 7-1
            new Pose2d(14.079, 2.628, new Rotation2d(Math.toRadians(120))),  // 6-1
            new Pose2d(12.370, 2.432, new Rotation2d(Math.toRadians(60)))    // 11-1
        );
    }
}