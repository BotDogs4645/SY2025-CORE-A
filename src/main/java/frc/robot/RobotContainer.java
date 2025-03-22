// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.logging.Logger;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.MechanismPosition;
import frc.robot.commands.CommandBuilder;
import frc.robot.commands.DriverAssist;
import frc.robot.commands.components.EndEffectorComponents;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Chute;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.vision.Localization;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandJoystick operatorPanel = new CommandJoystick(1);
    private final CommandXboxController secondJoystick = new CommandXboxController(2);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Localization visionSubsustem = new Localization(
            drivetrain::addVisionMeasurement,
            drivetrain::getState);
    public final EndEffector endEffector = new EndEffector();
    public final Elevator elevator = new Elevator(endEffector::isSafeToElevate);
    public static boolean hasBeenDeployed = false;
    public final Chute chute = new Chute();

    private final LoggedDashboardChooser<Command> autoChooser;

    public RobotContainer() {

        NamedCommands.registerCommand("deploy", CommandBuilder.deploy(chute, endEffector, elevator));
        NamedCommands.registerCommand("toMechanismPositionL1",
                CommandBuilder.toAutoMechanismPosition(endEffector, elevator, MechanismPosition.SCORE_L1));
        NamedCommands.registerCommand("toMechanismPositionL2",
                CommandBuilder.toAutoMechanismPosition(endEffector, elevator, MechanismPosition.SCORE_L2));
        NamedCommands.registerCommand("toMechanismPositionL3",
                CommandBuilder.toAutoMechanismPosition(endEffector, elevator, MechanismPosition.SCORE_L3));
        NamedCommands.registerCommand("toMechanismPositionL4",
                CommandBuilder.toAutoMechanismPosition(endEffector, elevator, MechanismPosition.SCORE_L4));
        NamedCommands.registerCommand("spit", EndEffectorComponents.spit(endEffector));
        NamedCommands.registerCommand("score", EndEffectorComponents.score(endEffector));
        NamedCommands.registerCommand("intakeSequence", CommandBuilder.intakeSequence(chute, endEffector, elevator));

        drivetrain.createAutoBuilder();
        autoChooser = new LoggedDashboardChooser<Command>("Auto Chooser", AutoBuilder.buildAutoChooser());

        configureBindings();

        SmartDashboard.putData("Auto Chooser", autoChooser.getSendableChooser());
        
    }

    private void configureBindings() {
        drivetrain.registerTelemetry(Telemetry::telemeterizeSwerve);

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                                   // negative Y
                                                                                                   // (forward)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                                    // negative X (left)
                ));

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(
        // () -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(),
        // -joystick.getLeftX()))));
        // joystick.x().onTrue(
        // Commands.runOnce(() -> {
        // Pose2d resetPose = new Pose2d(0, 0, new Rotation2d(0));
        // drivetrain.resetPose(resetPose);
        // System.out.println("Resetting position");
        // }));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        

        joystick.leftTrigger().whileTrue(drivetrain.defer(() -> DriverAssist.reefPathfindCommand(drivetrain, true, isBlueAlliance())));
        joystick.rightTrigger().whileTrue(drivetrain.defer(() -> DriverAssist.reefPathfindCommand(drivetrain, false, isBlueAlliance())));

        // joystick.leftTrigger().whileTrue(new AutoAlignCommand(drivetrain, new
        // Pose2d(3.38, 4.09, new Rotation2d(0))));

        joystick.button(7).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        joystick.button(8).onTrue(
                CommandBuilder.deploy(chute, endEffector, elevator));

        joystick.leftBumper().onTrue(CommandBuilder.intakeOrProcessor(chute, endEffector, elevator));
        joystick.rightBumper().whileTrue(EndEffectorComponents.spit(endEffector));

        operatorPanel.button(1).onTrue(
                CommandBuilder.toMechanismPosition(endEffector, elevator, MechanismPosition.SCORE_L1));

        operatorPanel.button(2).onTrue(
                CommandBuilder.toMechanismPosition(endEffector, elevator, MechanismPosition.SCORE_L2));

        operatorPanel.button(3).onTrue(
                CommandBuilder.toMechanismPosition(endEffector, elevator, MechanismPosition.SCORE_L3));

        operatorPanel.button(4).onTrue(
                CommandBuilder.toMechanismPosition(endEffector, elevator, MechanismPosition.SCORE_L4));

        operatorPanel.button(5).onTrue(
                CommandBuilder.toMechanismPosition(endEffector, elevator, MechanismPosition.DEALGAE_GROUND)
                        .andThen(CommandBuilder.intakeAlgae(endEffector))
        // new InstantCommand(endEffector::decreaseOffset)
        );

        operatorPanel.button(6).onTrue(
                CommandBuilder.toMechanismPosition(endEffector, elevator, MechanismPosition.DEALGAE_LOW)
                        .andThen(CommandBuilder.intakeAlgae(endEffector))
        // new InstantCommand(endEffector::decreaseOffset)
        );

        operatorPanel.button(7).onTrue(
                CommandBuilder.toMechanismPosition(endEffector, elevator, MechanismPosition.DEALGAE_HIGH)
                        .andThen(CommandBuilder.intakeAlgae(endEffector))
        // new InstantCommand(endEffector::increaseOffset)
        );

        // operatorPanel.button(8).onTrue(
        // CommandBuilder.toMechanismPosition(endEffector, elevator,
        // MechanismPosition.PROCESSOR)
        // .andThen(Commands.waitSeconds(2))
        // .andThen(new InstantCommand(() -> endEffector.setWheelCoast()))
        // );

        operatorPanel.button(9).toggleOnTrue(
                CommandBuilder.intakeSequence(chute, endEffector, elevator));

        // operatorPanel.button(9).onTrue(
        // EndEffectorComponents.intakeCoral(endEffector)
        // );

        // operatorPanel.button(9).onTrue(
        // CommandBuilder.intakeAlgaeLow(endEffector, elevator)
        // );

        operatorPanel.button(10).onTrue(
                CommandBuilder.toggleDeploy(chute, endEffector, elevator));
    }

    public boolean isBlueAlliance() {
        var alliance = DriverStation.getAlliance();
        if(alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Blue;
        }
        return true;
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}