// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Drivetrain;

public class CMD_ResetGyro extends CommandBase {
  /** Creates a new CMD_ResetGyro. */
  SUB_Drivetrain m_drivetrain;
  public CMD_ResetGyro(SUB_Drivetrain p_drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
  m_drivetrain = p_drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.resetGyro();
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
