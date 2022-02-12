/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;

public class Lift extends SubsystemBase {

  private double kP, kI, kD, kF;
  private int curr_position;
  //private boolean okToDescend, decsending;
  private static final CANSparkMax liftLeftSparkMax = new CANSparkMax(Constants.LiftConstants.LiftLeftSparkMax, MotorType.kBrushless);
  private static final CANSparkMax liftRightSparkMax = new CANSparkMax(Constants.LiftConstants.LiftRightSSparkMax, MotorType.kBrushless);
  private final SparkMaxPIDController leftSmartMaxPIDController;
  private final RelativeEncoder leftSmartMaxEncoder;
  private final SparkMaxPIDController rightSmartMaxPIDController;
  private final RelativeEncoder rightSmartMaxEncoder;

  /**
   * Creates a new lift.
   */
  public Lift() {

    //okToDescend = false;
    //decsending = false;
    kP = 0.025;
    kI = 0;
    kD = 0;
    kF = 0;

    liftLeftSparkMax.restoreFactoryDefaults();
    liftRightSparkMax.restoreFactoryDefaults();

    liftLeftSparkMax.setInverted(false);
    liftRightSparkMax.setInverted(false);

    // right(3) should follow left(5), this sets the voltage to mirror, all other settings must be distinctly set on both controllers.
    liftRightSparkMax.follow(liftLeftSparkMax);

    liftLeftSparkMax.set(0.0);

    //configure the left lift motor
    liftLeftSparkMax.clearFaults();
    liftLeftSparkMax.enableVoltageCompensation(12);
    liftLeftSparkMax.setSmartCurrentLimit(40, 20, 10);
    liftLeftSparkMax.setOpenLoopRampRate(1.5);
    liftLeftSparkMax.setIdleMode(IdleMode.kBrake);
    liftLeftSparkMax.setSoftLimit(SoftLimitDirection.kForward, 30);
    liftLeftSparkMax.enableSoftLimit(SoftLimitDirection.kForward, true);
    liftLeftSparkMax.setSoftLimit(SoftLimitDirection.kReverse, 30);
    liftLeftSparkMax.enableSoftLimit(SoftLimitDirection.kReverse, true);
    liftLeftSparkMax.setClosedLoopRampRate(1.5);
    liftLeftSparkMax.setSecondaryCurrentLimit(120, 30);

    //initialize Left PID and encoder
    leftSmartMaxPIDController = liftLeftSparkMax.getPIDController();
    leftSmartMaxEncoder = liftLeftSparkMax.getEncoder();
    leftSmartMaxPIDController.setI(kI);
    leftSmartMaxPIDController.setP(kP);
    leftSmartMaxPIDController.setD(kD);
    leftSmartMaxPIDController.setFF(kF);
    leftSmartMaxPIDController.setSmartMotionMaxVelocity(20, 1);
    

    //configure the right lift motor
      // liftLeftSparkMax.configPeakCurrentLimit(70, 30);
    liftRightSparkMax.clearFaults();
    liftRightSparkMax.enableVoltageCompensation(12);
    liftRightSparkMax.setSmartCurrentLimit(40, 20, 10);
    liftRightSparkMax.setOpenLoopRampRate(1.5);
    liftRightSparkMax.setIdleMode(IdleMode.kBrake);
    liftRightSparkMax.setSoftLimit(SoftLimitDirection.kForward, 30);
    liftRightSparkMax.enableSoftLimit(SoftLimitDirection.kForward, true);
    liftRightSparkMax.setSoftLimit(SoftLimitDirection.kReverse, 30);
    liftRightSparkMax.enableSoftLimit(SoftLimitDirection.kReverse, true);
    liftRightSparkMax.setClosedLoopRampRate(1.5);
    liftRightSparkMax.setSecondaryCurrentLimit(120, 30);

    //initialize Right PID and encoder
    rightSmartMaxPIDController = liftLeftSparkMax.getPIDController();
    rightSmartMaxEncoder = liftLeftSparkMax.getEncoder();
    rightSmartMaxPIDController.setI(kI);
    rightSmartMaxPIDController.setP(kP);
    rightSmartMaxPIDController.setD(kD);
    rightSmartMaxPIDController.setFF(kF);
    rightSmartMaxPIDController.setSmartMotionMaxVelocity(20, 1);
    rightSmartMaxEncoder.setInverted(true);


    //pre-flight checklist to make sure lift is all the way @ bottom
    curr_position = Constants.LiftPositions.Home;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //okToDescend = (curr_position == Constants.liftPositions.Top && Math.abs(liftLeftSparkMax.getClosedLoopError()) < 3000);
  }

  public void up() {
    curr_position = Constants.LiftPositions.Top;
    this.setPosition(Constants.LiftPositions.Top);
  }

  public void down() {
    //if(okToDescend || decsending){
      //decsending = true;
      curr_position -= Constants.LiftPositions.Increment;
      if(curr_position > Constants.LiftPositions.Increment){
        this.setPosition(curr_position);
      }
      else{
        curr_position = Constants.LiftPositions.Increment; 
        this.setPosition(curr_position);
      }
    //}
  }

  private void setPosition(double position) {
    leftSmartMaxEncoder.setPosition(position);
    //We may have to negate this, I'm hoping the invertion will take care of it.
    rightSmartMaxEncoder.setPosition(position);
  }

  public void stop() {
    //only set the left motor, the right motor is set to follow
    liftLeftSparkMax.set(0.0);
  }

}