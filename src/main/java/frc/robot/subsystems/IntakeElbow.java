/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class IntakeElbow extends SubsystemBase {

  private CANSparkMax sparkMax;
  private SparkMaxPIDController pidController;
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  private double home, down;
  private float kForwardSoftLimit, kReverseSoftLimit;
  private RelativeEncoder encoder;
  private boolean done = false;


  /**
   * Creates a new IntakeElbow.
   */
  
  public IntakeElbow() {
    // initialize motor
    sparkMax = new CANSparkMax(Constants.IntakeConstants.IntakeElbowSparkMax, MotorType.kBrushless);

    /**
     * The restoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    sparkMax.restoreFactoryDefaults();

    /**
     * In order to use PID functionality for a controller, a CANPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    pidController = sparkMax.getPIDController();
    encoder = sparkMax.getEncoder();
    
    // PID coefficients
    kP = 0.015;
    kI = 0;
    kD = 0.005; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    kForwardSoftLimit = -100;
    kReverseSoftLimit = 0;
    home = 0;
    down = kForwardSoftLimit;

    // set PID coefficients
    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(kIz);
    pidController.setFF(kFF);
    pidController.setOutputRange(kMinOutput, kMaxOutput);

    // sparkMax.setSoftLimit(SoftLimitDirection.kForward, kForwardSoftLimit);
    // sparkMax.setSoftLimit(SoftLimitDirection.kReverse, kReverseSoftLimit);

    // sparkMax.enableSoftLimit(SoftLimitDirection.kForward, true);
    // sparkMax.enableSoftLimit(SoftLimitDirection.kReverse, true);

    sparkMax.setSmartCurrentLimit(40, 20, 10);
    sparkMax.clearFaults();
    sparkMax.enableVoltageCompensation(12);
    sparkMax.setIdleMode(IdleMode.kBrake);
    sparkMax.setClosedLoopRampRate(1.0);
    sparkMax.setSecondaryCurrentLimit(95, 250);

    encoder.setPosition(0.0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (Math.abs(encoder.getPosition() / kReverseSoftLimit) > 0.90){
      done = true;
    }

    SmartDashboard.putNumber("elbow position", encoder.getPosition());
  }

  public void home(){
    System.out.println("home");
    pidController.setReference(home, ControlType.kPosition);
  }

  public boolean down(){
    System.out.println("down");
    pidController.setReference(down, ControlType.kPosition);
    return done;
  }

}
