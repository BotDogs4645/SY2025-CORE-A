// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.lang.ModuleLayer.Controller;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.Choreo;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.CommandBuilder;
import frc.robot.commands.components.ElevatorDown;
import frc.robot.commands.components.ElevatorUp;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.vision.Localization;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final CommandXboxController joystick = new CommandXboxController(0);

//     public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
//     public final Localization visionSubsustem = new Localization(
//         drivetrain::addVisionMeasurement,
//         drivetrain::getState
//     );
//     public final AutoFactory autoFactory = drivetrain.createAutoFactory(Telemetry::telemeterizeTrajectory);

    public Elevator elevator;

    private final AutoChooser autoChooser = new AutoChooser();

    public RobotContainer() {
        // autoChooser.addCmd("First Path", this::firstPath);
        // autoChooser.addCmd("Second Path", this::secondPath);
        // autoChooser.addCmd("Dath", this::dathPath);

        SmartDashboard.putData("Auto Chooser", autoChooser);
        elevator = new Elevator();

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // // and Y is defined as to the left according to WPILib convention.
        // drivetrain.setDefaultCommand(
        //         // Drivetrain will execute this command periodically
        //         drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
        //                                                                                            // negative Y
        //                                                                                            // (forward)
        //                 .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
        //                 .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with
        //                                                                             // negative X (left)
        //         ));

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(
        //         () -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));
        // joystick.x().onTrue(
        //         Commands.runOnce(() -> {
        //             Pose2d resetPose = new Pose2d(0, 0, new Rotation2d(0));
        //             drivetrain.resetPose(resetPose);
        //             System.out.println("Resetting position");
        //         }));


        joystick.x().onTrue(CommandBuilder.ElevatorToLevel(elevator, 1));
        joystick.y().onTrue(Commands.runOnce(() -> {
                elevator.setCoast();
            }));
        joystick.a().onTrue(CommandBuilder.ElevatorToLevel(elevator, 2));

        joystick.povUp().whileTrue(new ElevatorUp(elevator, joystick));
        joystick.povDown().whileTrue(new ElevatorDown(elevator, joystick));
        joystick.b().onTrue(Commands.runOnce(() -> {
            elevator.setPosition(0);
        }));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // // reset the field-centric heading on left bumper press
        // joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // drivetrain.registerTelemetry(Telemetry::telemeterizeSwerve);
    }

//     public Command firstPath() {
//         return Commands.sequence(
//                 new InstantCommand(() -> drivetrain
//                         .resetPose(Choreo.loadTrajectory("firstpath").get().getInitialPose(false).orElse(null))),
//                 autoFactory.trajectoryCmd("firstpath"));

//     }

//     public Command secondPath() {
//         return Commands.sequence(
//                 new InstantCommand(() -> drivetrain
//                         .resetPose(Choreo.loadTrajectory("secondpath").get().getInitialPose(false).orElse(null))),
//                 autoFactory.trajectoryCmd("secondpath"));
//     }

//     public Command dathPath() {
//         return Commands.sequence(
//                 new InstantCommand(() -> drivetrain
//                         .resetPose(Choreo.loadTrajectory("dath").get().getInitialPose(false).orElse(null))),
//                 autoFactory.trajectoryCmd("dath"));
//     }

    public Command getAutonomousCommand() {
        return autoChooser.selectedCommandScheduler();
    }
}
