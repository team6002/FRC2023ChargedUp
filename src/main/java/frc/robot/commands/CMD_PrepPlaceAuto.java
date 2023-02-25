// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.GlobalVariables;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.SUB_Elbow;
import frc.robot.subsystems.SUB_Elevator;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.SUB_Wrist;


public class CMD_PrepPlaceAuto extends SequentialCommandGroup {
  public CMD_PrepPlaceAuto(SUB_Elevator p_elevator, SUB_Elbow p_elbow, SUB_Wrist p_wrist, SUB_Intake p_intake, GlobalVariables p_variables) {

    addCommands(
      new ParallelCommandGroup(
        new CMD_ElevatorSetPosition(p_elevator, 12),
        new SequentialCommandGroup(
          new CMD_ElbowSetPosition(p_elbow, 215),
          new CMD_ElevatorSafteyCheck(p_elevator)
      ),
        new SequentialCommandGroup(
          new CMD_CheckWristSafe(p_elbow, p_elevator),
          new CMD_WristSetPosition(p_wrist, WristConstants.kWristShelf)
        )
      )
    );
  }
}
