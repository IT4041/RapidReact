// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IntakeElbow;
import frc.robot.subsystems.IntakeWheels;

public class AutoTest2 extends SequentialCommandGroup {

        private DriveTrain m_drivetrain;
        private IntakeElbow m_intakeElbow;
        private IntakeWheels m_intakeWheels;
        private Indexer m_Indexer;
        private Trajectory m_Trajectory1;
        private Trajectory m_Trajectory2;

        /** Creates a new AutoTest2. */
        public AutoTest2(DriveTrain in_drivetrain, IntakeElbow in_intakeelbow, IntakeWheels in_intakeWheels, Indexer in_Indexer,
                        Trajectory in_trajectory1, Trajectory in_trajectory2) {

                // Add your commands in the addCommands() call, e.g.
                // addCommands(new FooCommand(), new BarCommand());
                m_drivetrain = in_drivetrain;
                m_intakeElbow = in_intakeelbow;
                m_intakeWheels = in_intakeWheels;
                m_Indexer = in_Indexer;
                m_Trajectory1 = in_trajectory1;
                m_Trajectory2 = in_trajectory2;

                var table = NetworkTableInstance.getDefault().getTable("troubleshooting");
                var leftReference = table.getEntry("left_reference");
                var leftMeasurement = table.getEntry("left_measurement");
                var rightReference = table.getEntry("right_reference");
                var rightMeasurement = table.getEntry("right_measurement");

                var leftController = new PIDController(DriveConstants.kPDriveVel, 0, 0);
                var rightController = new PIDController(DriveConstants.kPDriveVel, 0, 0);

                RamseteCommand ramseteCommand1 = new RamseteCommand(
                                m_Trajectory1,
                                m_drivetrain::getPose,
                                new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
                                new SimpleMotorFeedforward(
                                                DriveConstants.ksVolts,
                                                DriveConstants.kvVoltSecondsPerMeter,
                                                DriveConstants.kaVoltSecondsSquaredPerMeter),
                                DriveConstants.kDriveKinematics,
                                m_drivetrain::getWheelSpeeds,
                                leftController,
                                rightController,
                                // RamseteCommand passes volts to the callback
                                (leftVolts, rightVolts) -> {
                                        m_drivetrain.tankDriveVolts(leftVolts, rightVolts);

                                        leftMeasurement.setNumber(m_drivetrain.getWheelSpeeds().leftMetersPerSecond);
                                        leftReference.setNumber(leftController.getSetpoint());

                                        rightMeasurement.setNumber(m_drivetrain.getWheelSpeeds().rightMetersPerSecond);
                                        rightReference.setNumber(rightController.getSetpoint());
                                },
                                m_drivetrain);

                RamseteCommand ramseteCommand2 = new RamseteCommand(
                                m_Trajectory2,
                                m_drivetrain::getPose,
                                new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
                                new SimpleMotorFeedforward(
                                                DriveConstants.ksVolts,
                                                DriveConstants.kvVoltSecondsPerMeter,
                                                DriveConstants.kaVoltSecondsSquaredPerMeter),
                                DriveConstants.kDriveKinematics,
                                m_drivetrain::getWheelSpeeds,
                                leftController,
                                rightController,
                                // RamseteCommand passes volts to the callback
                                (leftVolts, rightVolts) -> {
                                        m_drivetrain.tankDriveVolts(leftVolts, rightVolts);

                                        leftMeasurement.setNumber(m_drivetrain.getWheelSpeeds().leftMetersPerSecond);
                                        leftReference.setNumber(leftController.getSetpoint());

                                        rightMeasurement.setNumber(m_drivetrain.getWheelSpeeds().rightMetersPerSecond);
                                        rightReference.setNumber(rightController.getSetpoint());
                                },
                                m_drivetrain);

                // Reset odometry to the starting pose of the trajectory.
                m_drivetrain.resetOdometry(m_Trajectory1.getInitialPose());

                addCommands(
                                // new InstantCommand(m_intakeElbow::down,m_intakeElbow),
                                new InstantCommand(m_intakeWheels::on,m_intakeWheels),
                                new InstantCommand(m_Indexer::setAutoIndexOn, m_Indexer),
                                ramseteCommand1,
                                new InstantCommand(m_drivetrain::tankDriveVoltageStop, m_drivetrain),
                                new InstantCommand(m_drivetrain::setBrake, m_drivetrain),
                                //new WaitCommand(3),
                                new InstantCommand(m_drivetrain::setCoast, m_drivetrain),
                                ramseteCommand2,
                                new InstantCommand(m_drivetrain::tankDriveVoltageStop, m_drivetrain),
                                new InstantCommand(m_drivetrain::setBrake, m_drivetrain),
                                new WaitCommand(2),
                                new InstantCommand(m_drivetrain::setCoast, m_drivetrain)
                        );
        }
}
