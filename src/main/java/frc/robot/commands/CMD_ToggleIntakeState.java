// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.GlobalVariables;
import frc.robot.Constants.GlobalConstants;

public class CMD_ToggleIntakeState extends CommandBase {
  GlobalVariables m_variables;
  public CMD_ToggleIntakeState(GlobalVariables p_variables) {
    m_variables = p_variables;
  }

  @Override
  public void initialize() {
    if(m_variables.getIntakeState() == GlobalConstants.kConeMode){
      m_variables.setIntakeState(false);
    }else{
      m_variables.setIntakeState(true);
    }
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