// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Drivetrain;

public class CMD_SpinInPlace extends CommandBase {
  /** Creates a new CMD_SpinInPlace. */
  SUB_Drivetrain m_drivetrain;
  double m_angle;
  double m_tolerance = 7;
  public CMD_SpinInPlace(SUB_Drivetrain p_drivetrain, double p_angle) {
    m_drivetrain = p_drivetrain;
    m_angle = p_angle;
    addRequirements(m_drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.drive(0, 0, Math.copySign(.7, m_angle), false, false);
   }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0, 0, 0, false, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_drivetrain.getHeading() - m_angle) <= m_tolerance;
  }
}
