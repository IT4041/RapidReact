/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  private final CANSparkMax sparkMax1;
  private final CANSparkMax sparkMax2;
  private final SparkMaxPIDController pidController;
  private final RelativeEncoder encoder;
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, minRPM;
  private int accumulator = 0;
  private double velocity = 0.0;
  private double distanceRPMFactor = 45;
  private double rpmTolerance = 100;

  /**
   * Creates a new Shooter. This subsystem controls the shooter head
   */
  public Shooter() {

    sparkMax1 = new CANSparkMax(Constants.ShooterConstants.ShooterSparkMax1, MotorType.kBrushless);
    sparkMax2 = new CANSparkMax(Constants.ShooterConstants.ShooterSparkMax2, MotorType.kBrushless);
    pidController = sparkMax1.getPIDController();
    encoder = sparkMax1.getEncoder();

    sparkMax1.restoreFactoryDefaults();
    sparkMax2.restoreFactoryDefaults();

    sparkMax1.setIdleMode(IdleMode.kCoast);
    sparkMax2.setIdleMode(IdleMode.kCoast);

    sparkMax2.follow(sparkMax1, true);

    sparkMax1.enableVoltageCompensation(12);
    sparkMax2.enableVoltageCompensation(12);

    // PID coefficients
    kP = 0.00035; // kP = 0.0001;
    kI = 0.0;
    kD = 0.0048;//0.00035;
    kIz = 0;
    kFF = 0.0001675; // possible value for voltage pid
    kMaxOutput = 1;
    kMinOutput = -1;
    minRPM = -2000;

    // set PID coefficients
    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(kIz);
    pidController.setFF(kFF);
    pidController.setOutputRange(kMinOutput, kMaxOutput);

    SmartDashboard.putNumber("Calculated RPMS", 0);
    SmartDashboard.putBoolean("Ready to Shoot", false);
    SmartDashboard.putNumber("Actual RPMS", 0);
    SmartDashboard.putNumber("RPM diff", 0);
    SmartDashboard.putNumber("Distance Multiplier", distanceRPMFactor);

  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Actual RPMS", encoder.getVelocity());
    double dist = SmartDashboard.getNumber("Distance Multiplier", distanceRPMFactor);

    if (dist != distanceRPMFactor) {
      distanceRPMFactor = dist;
    }

  }

  // TODO: change pid to voltage
  // "I’ve generally found that good tuning of a velocity PID starts with
  // feed-forward
  // - as mentioned, set it to (max motor command)/(max output speed), and play
  // with it
  // to see how close you can get to your setpoint without any feedback. After
  // that,
  // bring in the rest of the PID to help reject disturbances and
  // get you even closer to the setpoint." gerthworm
  private double calculateRPMs(double distance) {

    double finalRPMS;

    // calculate rpms
    finalRPMS = distance * distanceRPMFactor;

    // use min rpms if calculated value is below min threshold
    finalRPMS = (finalRPMS < minRPM) ? minRPM : finalRPMS;
    SmartDashboard.putNumber("Calculated RPMS", -finalRPMS);
    return -finalRPMS;
  }

  public boolean readyToShoot() {
    boolean atSpeed = false;
    double measuredVelo = encoder.getVelocity();

    if (measuredVelo <= (this.velocity + rpmTolerance) && measuredVelo >= (this.velocity - this.rpmTolerance) && measuredVelo < -this.minRPM) {
      atSpeed = true;
      accumulator++;
    }
    SmartDashboard.putNumber("RPM diff", velocity - measuredVelo);
    SmartDashboard.putBoolean("Ready to Shoot", atSpeed && accumulator > 2);

    //TODO:restore calculation
    return atSpeed && accumulator > 2;
    //return true;
  }

  private void disablePID() {
    velocity = 0.0;
    pidController.setReference(velocity, ControlType.kVelocity);
    pidController.setP(0.0);
    pidController.setI(0.0);
    pidController.setD(0.0);
    pidController.setIZone(0.0);
    pidController.setFF(0.0);
  }

  private void enablePID() {
    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(kIz);
    pidController.setFF(kFF);
  }

  public void on(double distance) {
    this.enablePID();

    //TODO:restore calculation
    velocity = this.calculateRPMs(distance);
    
    
    
    pidController.setReference(velocity, ControlType.kVelocity);
  }

  public void off() {
    this.disablePID();
    accumulator = 0;
  }

  public void failSafeShoot(){
    this.off();
    sparkMax1.set(.5);
  }
}
