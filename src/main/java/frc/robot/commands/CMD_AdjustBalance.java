// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.SUB_Drivetrain;


public class CMD_AdjustBalance extends CommandBase {
  /** Creates a new CMD_AdjustBalance. */
  SUB_Drivetrain m_drivetrain;
  Timer m_timer;
  public CMD_AdjustBalance(SUB_Drivetrain p_drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = p_drivetrain;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    if (Math.abs(m_drivetrain.getVelocity()) > Units.inchesToMeters(2)){
      m_drivetrain.drive(Math.copySign(.3, m_drivetrain.getVelocity()), 0, 0, false, false);
    }else {
      m_drivetrain.drive(0, 0, 0, false, false);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    //use the Navx if availbe
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0, 0, 0, interrupted, interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.get() >= 2;
  }
}
