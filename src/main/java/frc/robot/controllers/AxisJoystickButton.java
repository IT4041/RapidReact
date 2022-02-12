// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controllers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


/** Add your docs here. */
public class AxisJoystickButton extends JoystickButton {
    private final XboxController m_xbox;
    private final int m_axis;
    private double m_targetVal;
    private ThresholdType m_thresholdType;
    
public static enum ThresholdType
{
        LESS_THAN, GREATER_THAN, EXACT, POV, DEADBAND;	
}

public AxisJoystickButton(XboxController in_xbox, int in_axis, double in_threshold, ThresholdType in_thresholdType) {
    super(in_xbox, in_axis);
    m_xbox = in_xbox;
    m_axis = in_axis;
    m_targetVal = in_threshold;
    m_thresholdType = in_thresholdType;
}

public boolean get() {
    switch (m_thresholdType) {
    case EXACT:
        //System.out.println("axis value: " + joy.getRawAxis(axis));
        return m_xbox.getRawAxis(m_axis) == m_targetVal;
    case LESS_THAN:
        return m_xbox.getRawAxis(m_axis) < m_targetVal;
    case GREATER_THAN:
        return m_xbox.getRawAxis(m_axis) > m_targetVal;
    case POV:
        return m_xbox.getPOV() == m_targetVal;
    case DEADBAND:
        return Math.abs(m_xbox.getRawAxis(m_axis)) > m_targetVal;
    default:
    return false;
    }
}

}
