// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Wrist;

public class CMD_WristSetPosition extends CommandBase {
  SUB_Wrist m_wrist;
  double m_position;
  double m_tolerance = 5;
  public CMD_WristSetPosition(SUB_Wrist p_wrist, double p_position) {
    m_wrist = p_wrist;
    m_position = p_position;
    addRequirements(m_wrist);
  }

  @Override
  public void initialize() {
    m_wrist.setWristOn(true);
    m_wrist.setReference(m_position);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    System.out.println("WRIST WRIST");
  }

  @Override
  public boolean isFinished() {
    return Math.abs(m_wrist.getWristPosition() - m_position) <= m_tolerance;
  }
}
