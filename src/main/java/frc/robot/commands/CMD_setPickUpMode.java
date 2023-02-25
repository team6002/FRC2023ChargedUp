// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.GlobalVariables;

public class CMD_setPickUpMode extends CommandBase {
  GlobalVariables m_variables;
  int m_pickMode;
  /** Creates a new CMD_PickUpMode. */
  public CMD_setPickUpMode(GlobalVariables p_variables, int p_pickMode) {
    m_variables = p_variables;
    m_pickMode = p_pickMode;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_variables.setPickMode(m_pickMode);
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
