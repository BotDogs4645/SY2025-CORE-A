package frc.robot.commands.components;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector;

public class EndEffectorComponents {
    public static Command intakeCoral(EndEffector endEffector) {
        return new Command() {
            @Override
            public void initialize() {
                endEffector.setWheelDutyCycle(0.2);
            }

            @Override
            public void execute() {
                if (endEffector.firstCoralSensorTripped()) {
                    endEffector.setWheelDutyCycle(0.05);
                }
            }

            @Override
            public void end(boolean interrupted) {
                endEffector.setWheelBrake();
            }

            @Override 
            public boolean isFinished() {
                return endEffector.secondCoralSensorTripped();
            }
        };
    }

    public static Command intakeAlgae(EndEffector endEffector) {
        return new Command() {
            @Override
            public void initialize() {
                endEffector.setWheelDutyCycle(0.2);
            }

            @Override
            public void execute() {
            }

            @Override
            public void end(boolean interrupted) {
            }
        };
    }

    public static Command spit(EndEffector endEffector) {
        return new Command() {
            @Override
            public void initialize() {
                endEffector.setWheelDutyCycle(0.2);
            }

            @Override
            public void execute() {
            }

            @Override
            public void end(boolean interrupted) {
                endEffector.setWheelCoast();
            }

            @Override 
            public boolean isFinished() {
                return false;
            }
        };
    }

    public static Command reverseCoral(EndEffector endEffector) {
        return new Command() {
            @Override
            public void initialize() {
            }

            @Override
            public void execute() {
               endEffector.setWheelDutyCycle(-0.08);
            }

            @Override
            public void end(boolean interrupted) {
                endEffector.setWheelBrake();
            }

            @Override 
            public boolean isFinished() {
                return endEffector.firstCoralSensorTripped() || !endEffector.secondCoralSensorTripped();
            }
        };
    }

    public static Command score(EndEffector endEffector) {
        return new Command() {
            private Supplier<Boolean> sensorProvider;

            @Override
            public void initialize() {
                if (endEffector.algaeSensorTripped()) {
                    endEffector.setWheelDutyCycle(-0.2);
                    sensorProvider = endEffector::algaeSensorTripped;
                } else {
                    endEffector.setWheelDutyCycle(0.2);
                    sensorProvider = endEffector::secondCoralSensorTripped;
                }
            }

            @Override
            public void execute() {
            }

            @Override
            public void end(boolean interrupted) {
                endEffector.setWheelDutyCycle(0);
            }

            @Override 
            public boolean isFinished() {
                return sensorProvider.get();
            }
        };
    }
}
