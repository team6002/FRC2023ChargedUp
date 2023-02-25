// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.GlobalVariables;
import frc.robot.Constants.GlobalConstants;

public class CMD_ToggleDropLevel extends CommandBase {
  /** Creates a new CMD_ToggleDropLevel. */
  GlobalVariables m_variables;
  public CMD_ToggleDropLevel(GlobalVariables p_variables) {
    m_variables = p_variables;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_variables.getDropLevel() == GlobalConstants.kElevator3rdLevel){
      m_variables.setDropLevel(GlobalConstants.kElevator1stLevel);
    }else m_variables.setDropLevel(m_variables.getDropLevel() + 1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
