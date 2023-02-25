// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.SUB_Elevator;

public class CMD_ElevatorSafteyCheck extends CommandBase {
  /** Creates a new CMD_ElevatorCheck. */
  SUB_Elevator m_elevator;
  double m_wantedPosition;
  double m_tolerance = ElevatorConstants.kElevatorTolerance;
  public CMD_ElevatorSafteyCheck(SUB_Elevator p_elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elevator = p_elevator;
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
    return m_elevator.getPosition() > ElevatorConstants.kElevatorPrep;
  }
}
