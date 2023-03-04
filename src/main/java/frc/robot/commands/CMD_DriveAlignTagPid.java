// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_LimeLight;

public class CMD_DriveAlignTagPid extends CommandBase {
  private SUB_Drivetrain m_drivetrain;
  private SUB_LimeLight m_limeLight;

  private final ProfiledPIDController xController;
  private final ProfiledPIDController yController;
  private final ProfiledPIDController turnController;

  private boolean end;

  private double xSpeed, ySpeed, turnSpeed;

  public CMD_DriveAlignTagPid(SUB_Drivetrain p_drivetrain, SUB_LimeLight p_limeLight) {
    m_drivetrain = p_drivetrain;
    m_limeLight = p_limeLight;

    xController = new ProfiledPIDController(Constants.AutoAlignConstants.driveKp,
      Constants.AutoAlignConstants.driveKi,
      Constants.AutoAlignConstants.driveKd,
      Constants.AutoAlignConstants.driveConstraints);

    yController = new ProfiledPIDController(Constants.AutoAlignConstants.driveKp,
      Constants.AutoAlignConstants.driveKi,
      Constants.AutoAlignConstants.driveKd,
      Constants.AutoAlignConstants.driveConstraints);

    turnController = new ProfiledPIDController(Constants.AutoAlignConstants.turnKp,
      Constants.AutoAlignConstants.turnKi,
      Constants.AutoAlignConstants.turnKd,
      Constants.AutoAlignConstants.turnConstraints);

    addRequirements(m_drivetrain, m_limeLight);
  }

  @Override
  public void initialize() {
    end = false;

    if (!m_limeLight.hasTarget()) {
      System.out.println("no limelight targets found, bailing");
      end = true;
      return;
    }

    // m_drivetrain.resetOdometryPose2d(m_limeLight.getRobotPoseInTargetSpace());

    xController.setGoal(Constants.AutoAlignConstants.goalPose.getX());
    yController.setGoal(Constants.AutoAlignConstants.goalPose.getY());
    turnController.setGoal(m_drivetrain.getAngle() - m_limeLight.getTargetYaw() + Constants.AutoAlignConstants.goalPose.getRotation().getDegrees());

    xController.setTolerance(.05);
    yController.setTolerance(.05);
    turnController.setTolerance(2);

    xController.reset(m_limeLight.getTargetX());
    yController.reset(m_limeLight.getTargetY());
    turnController.reset(m_drivetrain.getAngle());

    turnController.enableContinuousInput(-180, 180);
  }

  @Override
  public void execute() {
    if (end) {
      return;
    }

    if (!m_limeLight.hasTarget()) {
      System.out.println("Target lost, bailing");
      end = true;
    }

    if (xController.atGoal() && yController.atGoal() && turnController.atGoal()) {
    // if (xController.atGoal()) {
      System.out.println("At Goal " + Timer.getFPGATimestamp());
      end = true;
      return;
    }

    if (xController.atGoal()) {
      xSpeed = 0.;
    } else {
      xSpeed = xController.calculate(m_limeLight.getTargetX());
    }
    
    if (yController.atGoal()) {
      ySpeed = 0.;
    } else {
      ySpeed = yController.calculate(m_limeLight.getTargetY());
    }

    turnSpeed = turnController.calculate(m_drivetrain.getAngle());

    SmartDashboard.putNumber("AutoAlignXSpeed: ", xSpeed);
    SmartDashboard.putNumber("AutoAlignYSpeed: ", ySpeed);
    SmartDashboard.putNumber("AutoAlignTurnSpeed: ", turnSpeed);
    SmartDashboard.putNumber("AutoAlignTurnGoal", turnController.getGoal().position);

    m_drivetrain.drive(xSpeed, ySpeed, -turnSpeed, false, false);
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0.0, 0.0, 0.0, true, false);
  }

  @Override
  public boolean isFinished() {
    if (end) {
      System.out.println("Done! " + Timer.getFPGATimestamp());
    }

    return end;
  }
}