/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.subsystems.components.RangeSensors;

public class Indexer extends SubsystemBase {

  private final CANSparkMax m_indexerSparkMax = new CANSparkMax(Constants.IndexerConstants.IndexerSparkMax, MotorType.kBrushless);
  private RangeSensors m_RangeSensors;
  private boolean m_automate = false;
  private boolean m_bumped = false;

  public Indexer(RangeSensors in_RangeSensors) {
    m_RangeSensors = in_RangeSensors;

    m_indexerSparkMax.restoreFactoryDefaults();
    m_indexerSparkMax.clearFaults();
    // m_indexerSparkMax.setSmartCurrentLimit(40, 20, 10);
    // m_indexerSparkMax.enableVoltageCompensation(12);
    m_indexerSparkMax.setIdleMode(IdleMode.kBrake);
    //m_indexerSparkMax.setOpenLoopRampRate(1.0);

    SmartDashboard.putBoolean("Auto Indexing", false);
    SmartDashboard.putNumber("Ball Count", 0);
    SmartDashboard.putBoolean("addBall", false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Auto Indexing", this.m_automate);

    if (this.m_automate) {// true lets the indexer do it's thing
      if (m_RangeSensors.topClear()) {// no high ball so we can index
        
        if (m_RangeSensors.IntakeTriggered()) {// we have bottom ball and no top
          m_indexerSparkMax.set(0.8);// index
        } else {
          m_indexerSparkMax.set(0.0);// don't index we have no balls
        }
        this.m_bumped = false;
      } 
      else {
        // lift has a high ball
        if (!this.m_bumped) { // if we haven't bumperd back already
          this.bumpBack();
        }
        this.m_bumped = true;
      }
    }
  }

  public void shoot() {
    // lift balls fast for shooting
    this.m_automate = false;
    m_indexerSparkMax.set(0.95);
  }

  public void off(){
    m_indexerSparkMax.set(0.0);
  }

  private void bumpBack() {
    // this is a function to allow for moving the
    // balls slightly down the indexer to prevent
    // jamming of shooter head
    m_indexerSparkMax.set(-0.4);
    Timer.delay(0.1);
    m_indexerSparkMax.set(0.0);
 
  }

  public void reverseIndexer() {
    m_indexerSparkMax.set(-0.95);
  }

  public void setAutoIndexOn() {
    m_automate = true;
  }

  public void setAutoIndexOff() {
    m_automate = false;
  }

}
