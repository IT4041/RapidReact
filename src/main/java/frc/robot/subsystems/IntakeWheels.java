/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class IntakeWheels extends SubsystemBase {

  private static final CANSparkMax sparkMaxWheels = new CANSparkMax(Constants.IntakeConstants.IntakeWheelsSparkMax, MotorType.kBrushless); 
  private static final CANSparkMax sparkMaxFeeder = new CANSparkMax(Constants.IntakeConstants.IntakeFeederSparkMax, MotorType.kBrushless); 
  private boolean wheelsOn;
  /**
   * Creates a new IntakeWheels.
   */ 
  public IntakeWheels(XboxController assist) {

    sparkMaxWheels.restoreFactoryDefaults();
    sparkMaxWheels.clearFaults();
    sparkMaxWheels.setInverted(false);
    sparkMaxWheels.setSmartCurrentLimit(40, 20, 10);
    sparkMaxWheels.enableVoltageCompensation(12);
    sparkMaxWheels.setIdleMode(IdleMode.kBrake);
    sparkMaxWheels.setClosedLoopRampRate(1.0);
    sparkMaxWheels.setSecondaryCurrentLimit(95, 250);

    sparkMaxFeeder.restoreFactoryDefaults();
    sparkMaxFeeder.clearFaults();
    sparkMaxFeeder.setInverted(true);
    sparkMaxFeeder.setSmartCurrentLimit(40, 20, 10);
    sparkMaxFeeder.enableVoltageCompensation(12);
    sparkMaxFeeder.setIdleMode(IdleMode.kBrake);
    sparkMaxFeeder.setClosedLoopRampRate(1.0);
    sparkMaxFeeder.setSecondaryCurrentLimit(95, 250);

    wheelsOn = false; 
  }

@Override
  public void periodic() {
    // This method will be called once per scheduler run
   
  }

  public void on(){
    sparkMaxWheels.set(0.85);
    sparkMaxFeeder.set(0.7);
    wheelsOn = true;
  }

  public void reverse(){
    sparkMaxWheels.set(-0.85);
    sparkMaxFeeder.set(-0.7);
  }

  public void off(){
    sparkMaxWheels.set(0.0); 
    sparkMaxFeeder.set(0.0);
    wheelsOn = false;
  }

  public void returnToPrevState(){
    if(wheelsOn){
      this.on();
    }
    else{
      this.off();
    }
  }

}
