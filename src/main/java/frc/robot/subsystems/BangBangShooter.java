// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;

public class BangBangShooter extends SubsystemBase {

  private final BangBangController bangBangController;
  private final SimpleMotorFeedforward feedforward;
  private final CANSparkMax sparkMax1;
  private final CANSparkMax sparkMax2;
  private final RelativeEncoder encoder;
  private double minRPM;
  private int accumulator = 0;
  private double velocity = 0.0;
  private double distanceRPMFactor = 117.5;//82.5;// 67.5
  private double rpmTolerance = 25;
  private double kstatic = 0;
  private double kvelocity = 0.19;
  private double kacceleraton = 0.23;

  /** Creates a new BangBangShooter. */
  public BangBangShooter() {

    bangBangController = new BangBangController();
    feedforward = new SimpleMotorFeedforward(kstatic, kvelocity, kacceleraton);

    sparkMax1 = new CANSparkMax(Constants.ShooterConstants.ShooterSparkMax1, MotorType.kBrushless);
    sparkMax2 = new CANSparkMax(Constants.ShooterConstants.ShooterSparkMax2, MotorType.kBrushless);
    encoder = sparkMax1.getEncoder();

    sparkMax1.restoreFactoryDefaults();
    sparkMax2.restoreFactoryDefaults();

    sparkMax1.setIdleMode(IdleMode.kCoast);
    sparkMax2.setIdleMode(IdleMode.kCoast);

    sparkMax2.follow(sparkMax1, true);

    sparkMax1.enableVoltageCompensation(12);
    sparkMax2.enableVoltageCompensation(12);

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

    // TODO: check all conversions and unit and everything else before trying this
    // check relative encoder get velocity to make sure it's reporting the same things as encoder.getrate()
    sparkMax1.set(bangBangController.calculate(encoder.getVelocity(), velocity) + 0.9 * feedforward.calculate(velocity));
    encoder.getVelocityConversionFactor();

  }

  private double calculateRPMs(double distance) {

    double finalRPMS;
    double origin = minRPM;

    // calculate rpms
    finalRPMS = origin + (((distance - 120) / 12) * distanceRPMFactor);

    // use min rpms if calculated value is below min threshold
    finalRPMS = (finalRPMS < minRPM) ? minRPM : finalRPMS;
    SmartDashboard.putNumber("Calculated RPMS", finalRPMS);
    return finalRPMS;
  }

  public boolean readyToShoot() {
    boolean atSpeed = false;
    double measuredVelo = encoder.getVelocity();

    if (measuredVelo <= (velocity + rpmTolerance) && measuredVelo >= (velocity - rpmTolerance) && measuredVelo > 3300) {
      atSpeed = true;
      accumulator++;
    }
    SmartDashboard.putNumber("RPM diff", velocity - measuredVelo);
    SmartDashboard.putBoolean("Ready to Shoot", atSpeed && accumulator > 2);
    return atSpeed && accumulator > 2;
  }

  public void on(double distance) {
    velocity = this.calculateRPMs(distance);
  }

  public void off() {
    accumulator = 0;
  }

  public void failSafeShoot(){
    this.off();
    sparkMax1.set(.5);
  }
}
