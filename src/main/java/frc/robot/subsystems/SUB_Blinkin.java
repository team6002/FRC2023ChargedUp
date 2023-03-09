// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants;

public class SUB_Blinkin extends SubsystemBase {
  Spark m_blinkin;

  public SUB_Blinkin() {
    m_blinkin = new Spark(Constants.BlinkinConstants.kBlinkinPortId);
  }

  public void set(double pwm) {
    m_blinkin.set(pwm);
  }

  @Override
  public void periodic() {}
}