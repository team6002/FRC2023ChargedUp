// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Elevator;

public class CMD_ElevatorSetConstraints extends CommandBase {
  /** Creates a new CMD_ElevatorSetConstraints. */
  SUB_Elevator m_elevator;
  double m_acceleration;
  double m_velocity;
  public CMD_ElevatorSetConstraints(SUB_Elevator p_elevator, double p_velocity, double p_acceleration) {
    m_elevator = p_elevator;
    m_acceleration = p_acceleration;
    m_velocity = p_velocity;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevator.setElevatorConstraints(m_velocity, m_acceleration);
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
