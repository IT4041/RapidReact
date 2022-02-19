/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems.components;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

public class RangeSensors extends SubsystemBase {


   static double ballThresholdLiftBottom = 200;
   static double ballThresholdLiftTop = 200;
   static double ballThresholdIntake = 200;

  // Create instance of Time-Of_Flight driver for device 1 & 2
  private final TimeOfFlight rangeSensorIndexerBottom = new TimeOfFlight(Constants.RangeSensorConstants.TimeOfFlightLiftBottom);
  private final TimeOfFlight rangeSensorIndexerTop = new TimeOfFlight(Constants.RangeSensorConstants.TimeOfFlightLiftTop);
  private final TimeOfFlight rangeSensorIntake = new TimeOfFlight(Constants.RangeSensorConstants.TimeOfFlightIntake);

  /**
   * Creates a new RangeSensors.
   */
  public RangeSensors() {
    // Configure time of flight sensor for short ranging mode and sample
    // distance every 5 ms
    rangeSensorIndexerBottom.setRangingMode(RangingMode.Short, 1);
    rangeSensorIndexerTop.setRangingMode(RangingMode.Short, 1);
    rangeSensorIntake.setRangingMode(RangingMode.Short, 1);

    // function can be use to restrict the "field of view" of the sensor
    // rangeSensorIndexerTop.setRangeOfInterest(topLeftX, topLeftY, bottomRightX, bottomRightY);

  }

  @Override
  public void periodic() {

    SmartDashboard.putBoolean("rangeSensorLiftBottom triggered?", this.bottomTriggered());
    SmartDashboard.putBoolean("rangeSensorLiftTop clear?", this.topClear());
    SmartDashboard.putBoolean("rangeSensorIntake triggered?", this.IntakeTriggered());

  }

  public boolean bottomTriggered(){
    return rangeSensorIndexerBottom.getRange() <= ballThresholdLiftBottom;
  }

  public boolean topClear(){
    return rangeSensorIndexerTop.getRange() > ballThresholdLiftTop;
  }

  public boolean IntakeTriggered(){
    return rangeSensorIntake.getRange() <= ballThresholdIntake;
  }
  
}

