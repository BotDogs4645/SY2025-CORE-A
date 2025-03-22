// Original code by FRC 6328
// Modified for use by FRC 4645
// https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/template_projects/sources/vision/src/main/java/frc/robot/subsystems/vision
// Used under GPL v3 license

package frc.robot.subsystems.vision;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;

public class Limelight {
  private final DoubleArrayPublisher orientationPublisher;

  private final DoubleSubscriber latencySubscriber;
  private final DoubleSubscriber txSubscriber;
  private final DoubleSubscriber tySubscriber;
  private final DoubleArraySubscriber megatag1Subscriber;
  private final DoubleArraySubscriber megatag2Subscriber;

  public Limelight(String name) {
    var table = NetworkTableInstance.getDefault().getTable(name);
    orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();
    latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
    txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
    tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
    megatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
    megatag2Subscriber = table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});
  }

  public void updateInputs(LimelightInputs inputs, double rotation) {
    // Update connection status based on whether an update has been seen in the last
    // 250ms
    inputs.connected = ((RobotController.getFPGATime() - latencySubscriber.getLastChange()) / 1e3) < 250;

    // Update target observation
    inputs.latestTargetObservation = new TargetObservation(
        Rotation2d.fromDegrees(txSubscriber.get()),
        Rotation2d.fromDegrees(tySubscriber.get()));

    // Update orientation for MegaTag 2
    orientationPublisher.accept(
        new double[] {rotation, 0.0, 0.0, 0.0, 0.0, 0.0 });
    NetworkTableInstance.getDefault()
        .flush(); // Increases network traffic but recommended by Limelight

    // Read new pose observations from NetworkTables
    Set<Integer> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();

    // Read MegaTag1 observations
    for (var rawSample : megatag1Subscriber.readQueue()) {
      if (rawSample.value.length == 0) continue;
      for (int i = 11; i < rawSample.value.length; i += 7) {
        tagIds.add((int) rawSample.value[i]);
      }
      
      poseObservations.add(

      
          new PoseObservation(
              // Timestamp, based on server timestamp of publish and latency
              rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,

              // 3D pose estimate
              parsePose(rawSample.value),

              // Ambiguity, using only the first tag because ambiguity isn't applicable for multitag
              rawSample.value.length >= 18 ? rawSample.value[17] : 0.0,

              // Tag count
              (int) rawSample.value[7],

              // Average tag distance
              rawSample.value[9],

              // Observation type
              PoseObservationType.MEGATAG_1));
    }
   
    // Read MegaTag2 observations
    // for (var rawSample : megatag2Subscriber.readQueue()) {
    //   if (rawSample.value.length == 0)
    //     continue;
    //   for (int i = 11; i < rawSample.value.length; i += 7) {
    //     tagIds.add((int) rawSample.value[i]);
    //   }
    //   poseObservations.add(
    //       new PoseObservation(
    //           // Timestamp, based on server timestamp of publish and latency
    //           rawSample.timestamp * 1.0e-3 - rawSample.value[6],

    //           // 3D pose estimate
    //           parsePose(rawSample.value),

    //           // Ambiguity, zeroed because the pose is already disambiguated
    //           0.0,

    //           // Tag count
    //           (int) rawSample.value[7],

    //           // Average tag distance
    //           rawSample.value[9],

    //           // Observation type
    //           PoseObservationType.MEGATAG_2));
    // }

    // Save pose observations to inputs object
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    // Save tag IDs to inputs objects
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }

  /** Parses the 3D pose from a Limelight botpose array. */
  private static Pose3d parsePose(double[] rawLLArray) {
    return new Pose3d(
      rawLLArray[0],
      rawLLArray[1],
      rawLLArray[2],
      new Rotation3d(
        Units.degreesToRadians(rawLLArray[3]),
        Units.degreesToRadians(rawLLArray[4]),
        Units.degreesToRadians(rawLLArray[5])));
  }

  @AutoLog
  public static class LimelightInputs {
    public boolean connected = false;
    public TargetObservation latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
    public PoseObservation[] poseObservations = new PoseObservation[0];
    public int[] tagIds = new int[0];
  }

  /** Represents the angle to a simple target, not used for pose estimation. */
  public static record TargetObservation(
    Rotation2d tx,
    Rotation2d ty) {
  }

  public static enum PoseObservationType {
    MEGATAG_1,
    MEGATAG_2,
  }

  /** Represents a robot pose sample used for pose estimation. */
  public static record PoseObservation(
    double timestamp,
    Pose3d pose,
    double ambiguity,
    int tagCount,
    double averageTagDistance,
    PoseObservationType type
  ) {}
}