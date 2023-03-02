// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.GlobalVariables;
import frc.robot.Constants.GlobalConstants;
import frc.robot.commands.CMD_PlaceForwardsCone;
import frc.robot.commands.CMD_SpinInPlace;
import frc.robot.commands.CMD_Stow;
import frc.robot.commands.CMD_setDropLevel;
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
    SUB_FiniteStateMachine p_finiteStateMachine, SUB_Wrist p_wrist, GlobalVariables p_variables) {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new CMD_setDropLevel(p_variables, GlobalConstants.kElevator3rdLevel),
      new CMD_PlaceForwardsCone(p_elevator, p_intake, p_elbow, p_wrist, p_finiteStateMachine, p_variables),
      new WaitCommand(0.5),
      new ParallelCommandGroup(
        new CMD_Stow(p_elevator, p_intake, p_elbow, p_wrist, p_finiteStateMachine),
        new AUTO_DriveOverChargingStation(p_trajectories, p_drivetrain)
      ),
      new CMD_PlaceForwards(p_elevator, p_intake, p_elbow, p_wrist, p_finiteStateMachine, p_variables),
      new AUTO_DriveOverChargingStation(p_trajectories, p_drivetrain),
      new CMD_SpinInPlace(p_drivetrain, 180),
      new AUTO_DriveBackOnChargeStation(p_trajectories, p_drivetrain)
      // new CMD_AdjustBalance(p_drivetrain)
    );
  }
}
