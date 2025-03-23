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
import java.util.stream.IntStream;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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

    public static double gearRatio = 72;

    public static Distance coralThreshold = Millimeters.of(50);
    public static Distance algaeThreshold = Millimeters.of(30);

    public static Rotation2d safetyAngle = Rotation2d.fromRotations(0.39);
  }

  public static class PathPlannerConstants {
    public static RobotConfig config;

    static {
      try {
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
        RotationsPerSecondPerSecond.of(5));
  }

  public class ElevatorConstants {

    // max: 1.36m

    public static int leftMotorCANId = 15; // CANivore bus
    public static int rightMotorCANId = 16; // CANivore bus
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
    public static String[] limelightNames = { "limelight" };

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.2;
    public static double maxZError = 0.75;

    // Ignore MT2 readings when robot is rotationg too fast
    public static double maxAngluarVelocity = 2 * Math.PI; // Rad/s

    public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // No rotation data available

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] limelightStdDevFactors = { 1.0 };
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
    INTAKE(0.42709901067747524, 0),
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
    public static final Transform2d leftReefOffset = new Transform2d(0.66, -0.15, Rotation2d.kZero);
    public static final Transform2d rightReefOffset = new Transform2d(0.66, 0.15, Rotation2d.kZero);

    private static final Transform2d flip180 = new Transform2d(0, 0, Rotation2d.k180deg); 

    private static final AprilTagFieldLayout aprilTagLayout = 
      AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    public static final int[] blueReefTags = {18, 19, 20, 21, 22, 17};
    public static final int[] redReefTags = {10, 9, 8, 7, 6, 11};

    private static List<Pose2d> getTagPoses(int[] tags) {
      return IntStream.of(tags)
        .mapToObj(
          id -> aprilTagLayout.getTagPose(id).get().toPose2d()
        ).toList();
    }

    private static List<Pose2d> getReefPoses(List<Pose2d> tagPoses, Transform2d transform) {
      return tagPoses.stream().map(
        pose -> pose.transformBy(transform).plus(flip180)
      ).toList();
    }

    public static final List<Pose2d> blueTagPoses = getTagPoses(blueReefTags);
    public static final List<Pose2d> redTagPoses = getTagPoses(redReefTags);

    public static final List<Pose2d> redLeftReefPoses = getReefPoses(redTagPoses, leftReefOffset);
    public static final List<Pose2d> redRightReefPoses = getReefPoses(redTagPoses, rightReefOffset);
    public static final List<Pose2d> blueLeftReefPoses = getReefPoses(blueTagPoses, leftReefOffset);
    public static final List<Pose2d> blueRightReefPoses = getReefPoses(blueTagPoses, rightReefOffset);
  }
}