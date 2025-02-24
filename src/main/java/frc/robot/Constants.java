package frc.robot;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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

    // Multipliers to apply for MegaTag 2 observations`
    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor =
        Double.POSITIVE_INFINITY; // No rotation data available

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] limelightStdDevFactors = {1.0};
  }
  public static class ClimbConstants {
    public static int climbMotorID = 0;
    public static int chuteMotorID = 9;
    public static double climbSpeed = 0.5;
    public static double chuteSpeed = 0.5;

    private final TalonFXConfiguration talonFXConfigs;
    
    talonFXConfigs = new TalonFXConfiguration();
    // set Motion Magic settings
        MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)
        // set slot 0 gains
        Slot0Configs slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output
        chuteMotor.getConfigurator().apply(talonFXConfigs);
  }
}