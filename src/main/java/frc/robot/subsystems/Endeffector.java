package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class EndEffector extends SubsystemBase {
	private TalonFX mainMotor;
	private TalonFX pivotMotor;
	private AbsoluteEncoder pivotEncoder;
	private DigitalInput input;
	private TalonFXConfiguration talonConfig;

	// https://www.chiefdelphi.com/t/using-rev-through-bore-encoder-as-zeroing-encoder-on-swerve-drive/428855/2

	public EndEffector() {
		this.mainMotor = new TalonFX(0);
		this.pivotMotor = new TalonFX(0);
		DigitalInput input = new DigitalInput(1);
		DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(input);

		// should be abs rotation on startup
		Rotation2d rotationInitial = new Rotation2d(pivotEncoder.get());
		talonConfig = new TalonFXConfiguration().withFeedback(
				new FeedbackConfigs().withSensorToMechanismRatio(Constants.EndEffectorConstants.pivotGearRatio));
		var slot0Configs = talonConfig.Slot0;
		slot0Configs.kS = Constants.MotionMagicConstants.kS; // Add 0.25 V output to overcome static friction
		slot0Configs.kV = Constants.MotionMagicConstants.kV; // A velocity target of 1 rps results in 0.12 V output
		slot0Configs.kA = Constants.MotionMagicConstants.kA; // An acceleration of 1 rps/s requires 0.01 V output
		slot0Configs.kP = Constants.MotionMagicConstants.kP; // A position error of 2.5 rotations results in 12 V output
		slot0Configs.kI = Constants.MotionMagicConstants.kI; // no output for integrated error
		slot0Configs.kD = Constants.MotionMagicConstants.kD; // A velocity error of 1 rps results in 0.1 V output
		slot0Configs.kG = Constants.MotionMagicConstants.kG; // output to overcome gravity (output)

		// set Motion Magic settings
		var motionMagicConfigs = talonConfig.MotionMagic;
		motionMagicConfigs.MotionMagicCruiseVelocity = Constants.MotionMagicConstants.cruiseVelocity; // Target cruise
																										// velocity of
																										// 80 rps
		motionMagicConfigs.MotionMagicAcceleration = Constants.MotionMagicConstants.acceleration; // Target acceleration
																									// of 160 rps/s (0.5
																									// seconds)
		motionMagicConfigs.MotionMagicJerk = Constants.MotionMagicConstants.jerk; // Target jerk of 1600 rps/s/s (0.1
																					// seconds)

		pivotMotor.getConfigurator().apply(talonConfig);
		pivotMotor.setPosition(rotationInitial.getRotations());
	}

	// False for coral
	// True for Algae
	// Spins direction while being called
	public void setSpin(double speed) {
		mainMotor.set(speed);
	}

	public void setPivotPosition(int level) {
		switch (level) {
			case 1 :
				MotionMagicVoltage req1 = new MotionMagicVoltage(Constants.EndEffectorConstants.level1.getRotations());
				pivotMotor.setControl(req1);
			case 2 :
				MotionMagicVoltage req2 = new MotionMagicVoltage(Constants.EndEffectorConstants.level2.getRotations());
				pivotMotor.setControl(req2);
			case 3 :
				MotionMagicVoltage req3 = new MotionMagicVoltage(Constants.EndEffectorConstants.level3.getRotations());
				pivotMotor.setControl(req3);
			case 4 :
				MotionMagicVoltage req4 = new MotionMagicVoltage(Constants.EndEffectorConstants.level4.getRotations());
				pivotMotor.setControl(req4);
		}
	}

	public void rotate(double speed) {
		pivotMotor.set(speed);
	}

	public void stopSpin() {
		mainMotor.set(0);
	}

	public void stopPivot() {
		pivotMotor.set(0);
	}
}
