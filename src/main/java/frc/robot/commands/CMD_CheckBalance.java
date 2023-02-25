// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Drivetrain;

public class CMD_CheckBalance extends CommandBase {
  /** Creates a new CMD_CheckBalance. */
  SUB_Drivetrain m_drivetrain;
  double m_roll;
  double m_tolerance = 1;
  public CMD_CheckBalance(SUB_Drivetrain p_drivetrain, double p_roll) {
    m_drivetrain = p_drivetrain;
    m_roll = p_roll;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_drivetrain.getRoll() - m_roll) < m_tolerance;
  }
}
