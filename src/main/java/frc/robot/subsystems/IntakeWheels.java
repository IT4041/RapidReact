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
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class IntakeWheels extends SubsystemBase {

  private static final CANSparkMax sparkMax = new CANSparkMax(Constants.IntakeConstants.IntakeWheelsSparkMax, MotorType.kBrushless); 
  private boolean wheelsOn;
  /**
   * Creates a new IntakeWheels.
   */ 
  public IntakeWheels(XboxController assist) {

    sparkMax.restoreFactoryDefaults();
    sparkMax.clearFaults();
    sparkMax.setInverted(false);
    sparkMax.setSmartCurrentLimit(40, 20, 10);
    sparkMax.enableVoltageCompensation(12);
    sparkMax.setIdleMode(IdleMode.kBrake);
    sparkMax.setClosedLoopRampRate(1.5);
    sparkMax.setSecondaryCurrentLimit(120, 30);

    sparkMax.setSoftLimit(SoftLimitDirection.kForward, 30);
    sparkMax.enableSoftLimit(SoftLimitDirection.kForward, true);
    sparkMax.setSoftLimit(SoftLimitDirection.kReverse, 30);
    sparkMax.enableSoftLimit(SoftLimitDirection.kReverse, true);


    wheelsOn = false; 
  }

@Override
  public void periodic() {
    // This method will be called once per scheduler run
   
  }

  public void on(){
    sparkMax.set(0.85);
    wheelsOn = true;
  }

  public void reverse(){
    sparkMax.set(-0.85);
  }

  public void off(){
    sparkMax.set(0.0); 
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
