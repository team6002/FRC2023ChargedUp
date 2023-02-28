// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Blinkin;

public class CMD_BlinkinSetColor extends CommandBase {
  SUB_Blinkin m_blinkin;
  double m_pwm;
  public CMD_BlinkinSetColor(SUB_Blinkin p_blinkin, double p_pwm) {
    m_blinkin = p_blinkin;
    m_pwm = p_pwm;
  }

  @Override
  public void initialize() {
    m_blinkin.set(m_pwm);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
