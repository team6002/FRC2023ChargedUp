// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.GlobalVariables;
import frc.robot.Constants.GlobalConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_Elbow;
import frc.robot.subsystems.SUB_Elevator;
import frc.robot.subsystems.SUB_FiniteStateMachine;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.SUB_Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AUTO_BalanceStation extends SequentialCommandGroup {
  /** Creates a new AUTO_BalanceStation. */
  public AUTO_BalanceStation(AUTO_Trajectories p_trajectories, SUB_Drivetrain p_drivetrain,
    SUB_Elbow p_elbow, SUB_Elevator p_elevator, SUB_Intake p_intake, 
    SUB_FiniteStateMachine p_finiteStateMachine, SUB_Wrist p_wrist, GlobalVariables p_variables,
    CommandXboxController p_controller) {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new CMD_setDropLevel(p_variables, GlobalConstants.kElevator3rdLevel),
      new CMD_setPickUpMode(p_variables, GlobalConstants.kPickBackGroundMode),
      new CMD_setIntakeMode(p_variables, GlobalConstants.kConeMode),
      new CMD_selectIntakeCommand(p_variables),
      new CMD_PlaceForwardsCone(p_elevator, p_intake, p_elbow, p_wrist, p_finiteStateMachine, p_variables),
      new CMD_IntakeDrop(p_intake, p_variables),
      new WaitCommand(0.3),
      new ParallelDeadlineGroup(
        new AUTO_DriveOverChargingStation(p_trajectories, p_drivetrain),
        new SequentialCommandGroup(
          new CMD_StowGround(p_elevator, p_intake, p_elbow, p_wrist, p_finiteStateMachine),
          new CMD_IntakeGroundBackCube(p_elbow, p_elevator, p_intake, p_wrist, p_finiteStateMachine, p_variables),  
          new CMD_IntakeOn(p_intake, p_variables)
        )
      ),
      new ParallelDeadlineGroup(
        new CMD_SpinInPlace(p_drivetrain, 180),
        new CMD_IntakeCheck(p_intake, p_controller)
      ),
      new ParallelCommandGroup(
      new CMD_StowGround(p_elevator, p_intake, p_elbow, p_wrist, p_finiteStateMachine),
      new AUTO_DriveBackOnChargeStation(p_trajectories, p_drivetrain)
      ),
      //do a until hit angle and then run the set distance
      new CMD_setDropLevel(p_variables, 2),
      // new CMD_PlaceForwardsCube(p_elevator, p_intake, p_elbow, p_wrist, p_finiteStateMachine, p_variables),
      // new CMD_IntakeDrop(p_intake, p_variables),
      // new WaitCommand(.5),
      new CMD_Stow(p_elevator, p_intake, p_elbow, p_wrist, p_finiteStateMachine),
      new CMD_setIntakeMode(p_variables, GlobalConstants.kCubeMode),
      new CMD_IntakeDrop(p_intake, p_variables),
      new WaitCommand(.3),
      new CMD_AdjustBalanceBackwards(p_drivetrain),
      new WaitCommand(.5),
      new CMD_AdjustBalanceForwards(p_drivetrain)
    );
  }
}
