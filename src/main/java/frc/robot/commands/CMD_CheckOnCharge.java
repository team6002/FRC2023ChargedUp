// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Drivetrain;

public class CMD_CheckOnCharge extends CommandBase {
  /** Creates a new CMD_CheckRoll. */
  SUB_Drivetrain m_drivetrain;
  boolean m_finished = false;
  public CMD_CheckOnCharge(SUB_Drivetrain p_drivetrain) {
    m_drivetrain = p_drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_finished = false;
  }
;
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(m_drivetrain.getRoll()) > 8){
      m_finished = true;
    }else m_finished = false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      System.out.println("YOUR GOOD");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_finished;
  }
}
