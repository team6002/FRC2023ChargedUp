// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.GlobalVariables;
import frc.robot.Constants.AutoAlignConstants.AlignPosition;

public class CMD_setAlignPosition extends CommandBase {
  GlobalVariables m_variables;
  AlignPosition m_alignPosition;

  public CMD_setAlignPosition(GlobalVariables p_variables, AlignPosition p_alignPosition) {
    m_variables = p_variables;
    m_alignPosition = p_alignPosition;
  }

  @Override
  public void initialize() {
    m_variables.setAlignPosition(m_alignPosition);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
