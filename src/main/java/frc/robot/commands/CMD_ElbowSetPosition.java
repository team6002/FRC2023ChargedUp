// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Elbow;


public class CMD_ElbowSetPosition extends CommandBase {
    
  SUB_Elbow m_elbow;
  double m_position;
  double m_tolerance = 5;

  public CMD_ElbowSetPosition(SUB_Elbow p_elbow, double p_position) {
    m_elbow = p_elbow;
    m_position = p_position;
    addRequirements(m_elbow);
  }

  @Override
  public void initialize() {
    m_elbow.setElbowOn(true);
    m_elbow.setReference(m_position);
  }

  @Override
  public void execute() {
  
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return Math.abs(m_elbow.getPosition() - m_position) < m_tolerance;//checks to see if elbow is at the wanted position
  }
}
