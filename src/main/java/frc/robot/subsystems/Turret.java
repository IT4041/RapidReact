/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;

public class Turret extends SubsystemBase {

  private final CANSparkMax m_turretSparkMax = new CANSparkMax(Constants.TurrentConstants.TurretSparkMax, MotorType.kBrushless);
  private SparkMaxPIDController pidController;
  private RelativeEncoder m_encoder;

  // value that store the current position of turret
  private double current = 0;
  private boolean aquireTarget = false;

  // neo 550 ppr = 42
  
  // gearbox pulley 24 tooth
  // turret pully 210 tooth

  // gearbox to turret ratio : 8.75 (210 / 24)
  // gear box ratio : 100:1 (per mark)
  // motor to turret ratio  875:1 (8.75 * 100)
  // click to to turret revolutions : 36750 (875 * 42)
  // clicks per degree of rotation : 102.0833333; (36750 / 360)
  // degress of freedom : 140 (70 left, 70 right)
  // max click offset allowed per direction : 7146 (70 * 102.083333)

  private double clicksPerDegree = -2.5;
  private double m_xOffset = 0.0; // x offset reported by limelight
  private final int maxOffset = 180; // Maximum x offset allow
  private final int tolerance = 10; // clicks off target
  private double error = 2;

  public Turret() {

    m_turretSparkMax.restoreFactoryDefaults();
    m_turretSparkMax.clearFaults();
    m_turretSparkMax.setSmartCurrentLimit(40, 20, 10);
    m_turretSparkMax.enableVoltageCompensation(12);
    m_turretSparkMax.setIdleMode(IdleMode.kBrake);
    m_turretSparkMax.setClosedLoopRampRate(1);

    m_encoder = m_turretSparkMax.getEncoder();
    pidController = m_turretSparkMax.getPIDController();

    pidController.setFeedbackDevice(m_encoder);

    pidController.setP(0.021 ,0);
    pidController.setI(0.0, 0);
    pidController.setD(0.001, 0);
    pidController.setIZone(0.0, 0);
    pidController.setFF(0.0, 0);
    pidController.setOutputRange(-1, 1, 0);

    m_turretSparkMax.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_turretSparkMax.enableSoftLimit(SoftLimitDirection.kReverse, true);

    m_turretSparkMax.setSoftLimit(SoftLimitDirection.kForward, maxOffset);
    m_turretSparkMax.setSoftLimit(SoftLimitDirection.kReverse, -maxOffset);

    //pre-flight checklist to make sure turrret is face directly backwards
    m_encoder.setPosition(0.0);
    pidController.setSmartMotionAllowedClosedLoopError(tolerance,0);

    SmartDashboard.putBoolean("onTarget", false);
    SmartDashboard.putBoolean("Aquire Target", false);
  }

  @Override
  public void periodic() {

    SmartDashboard.putBoolean("onTarget", this.onTarget());
    SmartDashboard.putBoolean("Aquire Target", this.aquireTarget);
    SmartDashboard.putNumber("turret error", this.error);

    if(aquireTarget){//Bombardier notified turrent to target
      // TrackTarget returns the offset to the target in degrees
      // if limelight has a valid target, if no valid target is found
      // TrackTarget returns no offset(0.0)
      // 1.)add the offset to current position
      current = current + trackTarget();
      SmartDashboard.putNumber("Current", current);
      //Only apply changes that are less than 90 degrees off starting position
      //if target positions is greater than 90 return 90 with the proper sign(+/-)
      //current = Math.abs(current) <= maxOffset ? current : (Math.signum(current) * maxOffset);
      // 2.) update pid setpoint to new position

      //TODO:restore calculation
      //pidController.setReference(current, ControlType.kPosition);
      
      // 4.) update current positoin to position after adjustment and delay
      current = m_encoder.getPosition();
    }
    SmartDashboard.putNumber("Current", current);
  }
  // OI function *******************************************************************
  public void targetingEnabled(double in_XOffset){
    //Turret will auto-aim towards target
    this.aquireTarget = true;
    //get x offset from limelight
    m_xOffset = in_XOffset;
    SmartDashboard.putNumber("m_xOffset", m_xOffset);
  }

  public void targetingDisabled(){
    //Turret will stop auto-aiming towards target
    // and return to center back position
    this.aquireTarget = false;
    
    //reset x offset
    m_xOffset = 0.0;

    //recenter turret on back of robot
    current = 0;
    pidController.setReference(current, ControlType.kPosition);
  }
  // end OI functions *******************************************************************


  // vision functions *******************************************************************
  private double trackTarget()
  {
    SmartDashboard.putNumber("m_xOffset * dptp", m_xOffset * clicksPerDegree);
    // TrackTarget returns the offset to the target in turret pulses (+/-)  
    return m_xOffset * clicksPerDegree;
    
  }

  public boolean onTarget(){
    // is the pid reporting that on the setpoint within the tolerance
    
    //TODO:restore calculation
    this.error = m_xOffset;
    return Math.abs(error) < tolerance;
    // return true;
  }

  // end vision functions *******************************************************************

}
