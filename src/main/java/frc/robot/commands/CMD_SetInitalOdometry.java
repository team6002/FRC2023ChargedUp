// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class CMD_SetInitalOdometry extends CommandBase {
  /** Creates a new CMD_ResetOdometry. */
  DriveSubsystem m_drivetrain;
  Trajectory m_trajectory;

  public CMD_SetInitalOdometry(DriveSubsystem p_drivetrain, Trajectory p_trajectory) {
    m_drivetrain = p_drivetrain;
    m_trajectory = p_trajectory;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.resetOdometry(m_trajectory.getInitialPose());
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
