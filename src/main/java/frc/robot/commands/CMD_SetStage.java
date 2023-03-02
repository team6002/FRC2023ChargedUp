// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.GlobalVariables;

public class CMD_SetStage extends CommandBase {
  /** Creates a new CMD_SetHasItem. */
  GlobalVariables m_variables;
  int m_stage;
  public CMD_SetStage(GlobalVariables p_variables, int p_stage) {
    m_variables = p_variables;
    m_stage = p_stage;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_variables.setStage(m_stage);
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
