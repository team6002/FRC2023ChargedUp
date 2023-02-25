// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.GlobalVariables;

public class CMD_setDropLevel extends CommandBase {
  /** Creates a new CMD_setDropLevel. */
  GlobalVariables m_variables;
  int m_level;
  public CMD_setDropLevel(GlobalVariables p_variables, int p_level) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_variables = p_variables;
    m_level = p_level;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_variables.setDropLevel(m_level);
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
