// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.GlobalVariables;
import frc.robot.Constants.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.SUB_FiniteStateMachine.RobotState;;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CMD_PrepIntakeShelfBack extends SequentialCommandGroup {
  public CMD_PrepIntakeShelfBack(SUB_Elbow p_elbow, SUB_Elevator p_elevator, SUB_Intake p_intake, SUB_Wrist p_wrist,
   SUB_FiniteStateMachine p_finiteStateMachine, GlobalVariables p_variables, SUB_Limelight p_cameras
   ) {
    addCommands(
      new CMD_setCamera(p_cameras, CameraConstants.kDriveCam),
      new CMD_setState(p_finiteStateMachine, RobotState.PREPINTAKE),
      new CMD_IntakeOff(p_intake),
      new ParallelCommandGroup(
        new CMD_ElevatorSetPosition(p_elevator, ElevatorConstants.kElevatorShelfBack),
        new ParallelDeadlineGroup(
          new CMD_CheckWristPosition(p_wrist, WristConstants.kWristShelf),
          new CMD_ElbowSetPosition(p_elbow, ElbowConstants.kElbowUp),
          new SequentialCommandGroup( 
            new WaitCommand(.2),
            new CMD_CheckWristSafe(p_elbow, p_elevator),
            new CMD_WristSetPosition(p_wrist, WristConstants.kWristShelf)
          )
        )
      ),
      new CMD_ElbowSetPosition(p_elbow, ElbowConstants.kElbowShelfBackPrep)
    );
  }
}
