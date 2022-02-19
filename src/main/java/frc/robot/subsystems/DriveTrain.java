// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FalconDriveConstants;
import frc.robot.subsystems.components.NavX;

public class DriveTrain extends SubsystemBase {

  boolean isAuto = DriverStation.isAutonomous();

  // declare the motor crontrollers
  public final WPI_TalonFX frontRightTalon = new WPI_TalonFX(FalconDriveConstants.FrontRightTalon);
  public final WPI_TalonFX frontLeftTalon = new WPI_TalonFX(FalconDriveConstants.FrontLeftTalon);
  public final WPI_TalonFX backRightTalon = new WPI_TalonFX(FalconDriveConstants.BackRightTalon);
  public final WPI_TalonFX backLeftTalon = new WPI_TalonFX(FalconDriveConstants.BackLeftTalon);
  public final WPI_TalonFX topRightTalon = new WPI_TalonFX(FalconDriveConstants.TopRightTalon);
  public final WPI_TalonFX topLeftTalon = new WPI_TalonFX(FalconDriveConstants.TopLeftTalon);

  // using Navx as gyro
  private NavX m_NavX;

  // The motors on the left side of the drive.
  private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(frontLeftTalon, backLeftTalon,
      topLeftTalon);

  // The motors on the right side of the drive.
  private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(frontRightTalon, backRightTalon,
      topRightTalon);

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  // Network tables for odometry logging
  private NetworkTableEntry m_xEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("X");
  private NetworkTableEntry m_yEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Y");
  private NetworkTableEntry m_LeftEntry = NetworkTableInstance.getDefault().getTable("troubleshooting")
      .getEntry("Left");
  private NetworkTableEntry m_RightEntry = NetworkTableInstance.getDefault().getTable("troubleshooting")
      .getEntry("Right");
  private NetworkTableEntry m_RotEntry = NetworkTableInstance.getDefault().getTable("troubleshooting")
      .getEntry("Rotation");

  private int counter = 0;

  /** Creates a new DriveSubsystem. */
  public DriveTrain(NavX in_navX) {

    m_NavX = in_navX;

    frontRightTalon.setNeutralMode(NeutralMode.Coast);
    frontLeftTalon.setNeutralMode(NeutralMode.Coast);
    backRightTalon.setNeutralMode(NeutralMode.Coast);
    backLeftTalon.setNeutralMode(NeutralMode.Coast);
    topRightTalon.setNeutralMode(NeutralMode.Coast);
    topLeftTalon.setNeutralMode(NeutralMode.Coast);

    m_drive.setExpiration(1);
    m_drive.setSafetyEnabled(false);

    // supConfig values; true = current limiting is on, set to 40 amp max, motors
    // can run at 60 apms for 1 second then falls back to 40 amps
    SupplyCurrentLimitConfiguration supConfig = new SupplyCurrentLimitConfiguration(true, 40, 60, 1);
    frontRightTalon.configSupplyCurrentLimit(supConfig);
    frontLeftTalon.configSupplyCurrentLimit(supConfig);
    backRightTalon.configSupplyCurrentLimit(supConfig);
    backLeftTalon.configSupplyCurrentLimit(supConfig);
    topRightTalon.configSupplyCurrentLimit(supConfig);
    topLeftTalon.configSupplyCurrentLimit(supConfig);

    frontRightTalon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    frontLeftTalon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    backRightTalon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    backLeftTalon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    topRightTalon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    topLeftTalon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(m_NavX.getRotation2d());

  }

  @Override
  public void periodic() {

    if (DriverStation.isAutonomous()) {
      // Update the odometry in the periodic block
      m_odometry.update(m_NavX.getRotation2d(), this.getLeftDistance(), this.getRightDistance());

      if (counter % 100 == 0) {
        var translation = m_odometry.getPoseMeters().getTranslation();
        m_xEntry.setNumber(translation.getX());
        m_yEntry.setNumber(translation.getY());

        m_LeftEntry.setNumber(this.getLeftDistance());
        m_RightEntry.setNumber(this.getRightDistance());

        Rotation2d rot = m_NavX.getRotation2d();
        m_RotEntry.setNumber(rot.getDegrees());
      }
    }
    else{
      frontLeftTalon.setNeutralMode(NeutralMode.Brake);
      frontRightTalon.setNeutralMode(NeutralMode.Brake);
    }
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {

    // raw sensor per 100ms, scale to meters per sec
    double leftRate = topLeftTalon.getSelectedSensorVelocity() * (10.0 / Constants.FalconDriveConstants.cpr)
        * Constants.FalconDriveConstants.wheelcircumference;
    double rightRate = topRightTalon.getSelectedSensorVelocity() * (10.0 / Constants.FalconDriveConstants.cpr)
        * Constants.FalconDriveConstants.wheelcircumference;

    return new DifferentialDriveWheelSpeeds(leftRate, rightRate);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, m_NavX.getRotation2d());
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double FWD, double ROT) {
    m_drive.arcadeDrive(ROT*0.5,FWD);
  }

  /**
   * Stops the left and right sides of the drive directly with voltages.
   */
  public void tankDriveVoltageStop() {
    this.tankDriveVolts(0, 0);
  }

  public void setBrake() {
    frontRightTalon.setNeutralMode(NeutralMode.Brake);
    frontLeftTalon.setNeutralMode(NeutralMode.Brake);
  }

  public void setCoast() {
    frontRightTalon.setNeutralMode(NeutralMode.Coast);
    frontLeftTalon.setNeutralMode(NeutralMode.Coast);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(-rightVolts);
    m_drive.feed();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    topLeftTalon.setSelectedSensorPosition(0);
    topRightTalon.setSelectedSensorPosition(0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (this.getLeftDistance() + this.getRightDistance()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public WPI_TalonFX getLeftIntegratedEncoder() {
    return topLeftTalon;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public WPI_TalonFX getRightIntegratedEncoder() {
    return topRightTalon;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more
   * slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_NavX.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_NavX.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_NavX.getRate();
  }

  private double getLeftDistance() {
    return (topLeftTalon.getSelectedSensorPosition() * FalconDriveConstants.distancePerPulse);

  }

  private double getRightDistance() {
    return -(topRightTalon.getSelectedSensorPosition() * FalconDriveConstants.distancePerPulse);
  }
}
