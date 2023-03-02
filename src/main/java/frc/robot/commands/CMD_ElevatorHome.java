// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.SUB_Elevator;

public class CMD_ElevatorHome extends CommandBase {
  /** Creates a new CMD_ElevatorHome. */
  SUB_Elevator m_elevator;
  double m_position;
  boolean m_finished;
  public CMD_ElevatorHome(SUB_Elevator p_elevator) {
    m_elevator = p_elevator;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevator.setElevatorOn(false);
    m_elevator.setPower(-.2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.resetElevatorPosition();
    m_elevator.setElevatorOn(true);
    m_elevator.setReference(0);
    m_elevator.setElevatorConstraints(ElevatorConstants.kElevatorMaxVelocity, ElevatorConstants.kElevatorMaxAcceleration);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_elevator.getElevatorCurrent() >= 20);
  }
}
