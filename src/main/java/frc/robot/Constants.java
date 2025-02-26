package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;

public class Constants {
  public static class ChoreoConstants {
    public static PIDController xController = new PIDController(10.0, 0.0, 0.0);
    public static PIDController yController = new PIDController(10.0, 0.0, 0.0);
    public static PIDController headingController = new PIDController(7.5, 0.0, 0.0);
  }

  public static class VisionConstants {
    public static String[] limelightNames = { "limelight" };

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.75;

    // Ignore MT2 readings when robot is rotationg too fast
    public static double maxAngluarVelocity = 2 * Math.PI; // Rad/s

    public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    // Multipliers to apply for MegaTag 2 observations`
    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // No rotation data available

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] limelightStdDevFactors = { 1.0 };
  }

  public static class ClimbConstants {
    public static int climbMotorID = 0;
    public static int chuteMotorID = 9;
    public static double climbSpeed = 0.5;
    public static double chuteSpeed = 0.5;
    public static double angleToStow = 103; //(103(90+13) is the angle of the chute stowed up)
    public static double angleToClimb = 135; //(135 is an angle that we can play with during testing)
    public static int stowGearRatio = 200;
    public static int climbGearRatio = 200;
    public static double stowDeadband = 0.05;
    public static double climbDeadband = 0.05; 
    public static double stowBottomEndStop = 13/360.0*stowGearRatio; // The minimum position of the chute in rotations (13 is the angle in degrees of the chute at rest)
    public static double stowTopPoint = 103/360.0*stowGearRatio; // The maximum position of the chute in rotations (103(90+13) is the angle of the chute stowed up)
    public static double climbBottomEndStop = 0; //The minimum position of the climber in rotations
    public static double climbTopPoint = 135/360.0*climbGearRatio; // The maximum position of the climber in rotations (135 is an angle that would allow the climber to rotate fully away from the center)


    public static final TalonFXConfiguration talonFXConfigs;
    static {
      talonFXConfigs = new TalonFXConfiguration();
      // set Motion Magic settings
      MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;
      motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
      motionMagicConfigs.MotionMagicAcceleration = 100; // Target acceleration of 160 rps/s (0.8 seconds)
      motionMagicConfigs.MotionMagicJerk = 1000; // Target jerk of 1200 rps/s/s (0.1 seconds)
      // set slot 0 gains
      Slot0Configs slot0Configs = talonFXConfigs.Slot0;
      slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
      slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
      slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
      slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
      slot0Configs.kI = 0; // no output for integrated error
      slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    }
  }
}