// Original code by FRC 6328
// Modified for use by FRC 4645
// https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/template_projects/sources/vision/src/main/java/frc/robot/subsystems/vision
// Used under GPL v3 license

package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.vision.Limelight.PoseObservationType;

public class Localization extends SubsystemBase {
	
	private final VisionConsumer visionConsumer;
	private final Supplier<SwerveDriveState> stateSupplier;

	private final Limelight[] limelights;
	private final LimelightInputsAutoLogged[] inputs;
  	private final Alert[] disconnectedAlerts;

	public Localization(VisionConsumer visionConsumer, Supplier<SwerveDriveState> stateSupplier) {
		this.visionConsumer = visionConsumer;
		this.stateSupplier = stateSupplier;

		this.limelights = new Limelight[VisionConstants.limelightNames.length];
		this.inputs = new LimelightInputsAutoLogged[VisionConstants.limelightNames.length];
		this.disconnectedAlerts = new Alert[VisionConstants.limelightNames.length];

		for (int i = 0; i < VisionConstants.limelightNames.length; i++) {
			limelights[i] = new Limelight(VisionConstants.limelightNames[i]);
			inputs[i] = new LimelightInputsAutoLogged();
			disconnectedAlerts[i] = new Alert(
				"Limelight '" + VisionConstants.limelightNames[i] + "' is disconnected.", 
				AlertType.kWarning
			);
		}
	}

	@Override
	public void periodic() {
		SwerveDriveState state = stateSupplier.get();
		double rotation = state.Pose.getRotation().getDegrees();

		// Initialize logging summary values
		List<Pose3d> allTagPoses = new LinkedList<>();
		List<Pose3d> allRobotPoses = new LinkedList<>();
		List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
		List<Pose3d> allRobotPosesRejected = new LinkedList<>();
		List<Double> allLinearStds = new ArrayList<Double>();
		List<Double> allAngularStds = new ArrayList<Double>();

		for (int i = 0; i < limelights.length; i++) {
			limelights[i].updateInputs(inputs[i], rotation);
			Logger.processInputs("Vision/" + VisionConstants.limelightNames[i], inputs[i]);
			disconnectedAlerts[i].set(!inputs[i].connected);

			// Initialize logging values
			List<Pose3d> tagPoses = new LinkedList<>();
			List<Pose3d> robotPoses = new LinkedList<>();
			List<Pose3d> robotPosesAccepted = new LinkedList<>();
			List<Pose3d> robotPosesRejected = new LinkedList<>();

			// Process each detected tag ID
			for (int tagId : inputs[i].tagIds) {
				var tagPose = VisionConstants.aprilTagLayout.getTagPose(tagId);
				if (tagPose.isPresent()) {
					tagPoses.add(tagPose.get());
				}
			}

			// Loop over pose observations
			for (var observation : inputs[i].poseObservations) {
				// Reject poor pose data based on several criteria
				boolean rejectPose = 
					observation.tagCount() == 0 || 
					// Ambiguity is only applicable to single tag MT1 observations.
					(observation.tagCount() == 1 && observation.ambiguity() > VisionConstants.maxAmbiguity) ||
					Math.abs(observation.pose().getZ()) > VisionConstants.maxZError || 
					// Pose must be within field.
					observation.pose().getX() < 0.0 ||
					observation.pose().getX() > VisionConstants.aprilTagLayout.getFieldLength() ||
					observation.pose().getY() < 0.0 ||
					observation.pose().getY() > VisionConstants.aprilTagLayout.getFieldWidth() ||
					// MT2 is inaccurate when rotating quickly
					(observation.type() == PoseObservationType.MEGATAG_2 &&
						state.Speeds.omegaRadiansPerSecond > VisionConstants.maxAngluarVelocity);

				// Add pose to log
				robotPoses.add(observation.pose());
				if (rejectPose) {
					robotPosesRejected.add(observation.pose());
				} else {
					robotPosesAccepted.add(observation.pose());
				}

				// Skip rejected poses
				if (rejectPose) {
					continue;
				}

				// Calculate standard deviation for the vision data
				double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
				double linearStdDev = VisionConstants.linearStdDevBaseline * stdDevFactor;
				double angularStdDev = VisionConstants.angularStdDevBaseline * stdDevFactor;

				if (i < VisionConstants.limelightStdDevFactors.length) {
					linearStdDev *= VisionConstants.limelightStdDevFactors[i];
					angularStdDev *= VisionConstants.limelightStdDevFactors[i];
				}

				if (observation.type() == PoseObservationType.MEGATAG_2) {
					linearStdDev *= VisionConstants.linearStdDevMegatag2Factor;
					angularStdDev *= VisionConstants.angularStdDevMegatag2Factor;
				}

				allLinearStds.add(linearStdDev);
				allAngularStds.add(angularStdDev);

				// Send vision observation to the consumer
				visionConsumer.accept(
					observation.pose().toPose2d(),
					observation.timestamp(),
					//TODO: Test on Saturday
					VecBuilder.fill(0.9, 0.9, 0.9)
					//VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev)
				);
			}

			

			// Log camera data
			Logger.recordOutput(
				"Vision/" + VisionConstants.limelightNames[i] + "/TagPoses",
				tagPoses.toArray(new Pose3d[0])
			);
			Logger.recordOutput(
				"Vision/" + VisionConstants.limelightNames[i] + "/RobotPoses",
				robotPoses.toArray(new Pose3d[0])
			);
			Logger.recordOutput(
				"Vision/" + VisionConstants.limelightNames[i] + "/RobotPosesAccepted",
				robotPosesAccepted.toArray(new Pose3d[0])
			);
			Logger.recordOutput(
				"Vision/" + VisionConstants.limelightNames[i] + "/RobotPosesRejected",
				robotPosesRejected.toArray(new Pose3d[0])
			);
			
			Logger.recordOutput(
				"Vision/allLinearStds",
				allLinearStds.stream().mapToDouble(Double::doubleValue).toArray()
			);
			Logger.recordOutput(
				"Vision/allAngularStds",
				allAngularStds.stream().mapToDouble(Double::doubleValue).toArray()
			);
			
			// Aggregate data for final summary
			allTagPoses.addAll(tagPoses);
			allRobotPoses.addAll(robotPoses);
			allRobotPosesAccepted.addAll(robotPosesAccepted);
			allRobotPosesRejected.addAll(robotPosesRejected);
		}

		// Log summary data
		Logger.recordOutput(
			"Vision/Summary/TagPoses",
			allTagPoses.toArray(new Pose3d[0])
		);
		Logger.recordOutput(
			"Vision/Summary/RobotPoses",
			allRobotPoses.toArray(new Pose3d[0])
		);
		Logger.recordOutput(
			"Vision/Summary/RobotPosesAccepted",
			allRobotPosesAccepted.toArray(new Pose3d[0])
		);
		Logger.recordOutput(
			"Vision/Summary/RobotPosesRejected",
			allRobotPosesRejected.toArray(new Pose3d[0])
		);
	}

	@Override
	public void simulationPeriodic() {
		// Placeholder for simulation-specific logic (not implemented)
	}

	@FunctionalInterface
	public interface VisionConsumer {
		void accept(
			Pose2d visionRobotPoseMeters,
			double timestampSeconds,
			Matrix<N3, N1> visionMeasurementStdDevs
		);
	}
}