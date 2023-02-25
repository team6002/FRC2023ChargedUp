// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Intake;


public class CMD_IntakeSetPower extends CommandBase {
  SUB_Intake m_intake;
  double m_speed = 0;
  public CMD_IntakeSetPower(SUB_Intake p_intake, double speed) {
    m_intake = p_intake;
    m_speed = speed;
    addRequirements(m_intake);
  }

  @Override
  public void initialize() {
    m_intake.setPower(m_speed);
  }

  @Override
  public void execute() {

  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
