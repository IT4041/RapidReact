/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
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
    
    // PID coefficients
    kP = 0.025; 
    kI = 0;
    kD = 0.0001; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    kForwardSoftLimit = 0;
    kReverseSoftLimit = -90;
    home = 0;
    down = kReverseSoftLimit;

    // set PID coefficients
    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(kIz);
    pidController.setFF(kFF);
    pidController.setOutputRange(kMinOutput, kMaxOutput);

    sparkMax.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, kForwardSoftLimit);
    sparkMax.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, kReverseSoftLimit);

    sparkMax.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    sparkMax.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

    sparkMax.setClosedLoopRampRate(0.5);
    sparkMax.setSmartCurrentLimit(4, 30, 10);

  }

  @Override
  public void periodic() {
    // // This method will be called once per scheduler run
    encoder = sparkMax.getEncoder();
    if (Math.abs(encoder.getPosition() / kReverseSoftLimit) > 0.90){
      done = true;
    }
  }

  public void home(){
    pidController.setReference(home, ControlType.kPosition);
  }

  public boolean down(){
    pidController.setReference(down, ControlType.kPosition);
    return done;
  }

}
