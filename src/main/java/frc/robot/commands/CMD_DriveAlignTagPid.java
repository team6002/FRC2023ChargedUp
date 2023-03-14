// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_Limelight;

public class CMD_DriveAlignTagPid extends CommandBase {
  private SUB_Drivetrain m_drivetrain;
  private SUB_Limelight m_limeLight;

  private final ProfiledPIDController xController;
  private final ProfiledPIDController yController;
  private final PIDController turnController;

  private boolean end;

  private double xSpeed, ySpeed, turnSpeed;

  private Pose2d goalPose;

  public CMD_DriveAlignTagPid(SUB_Drivetrain p_drivetrain, SUB_Limelight p_limeLight) {
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

    turnController = new PIDController(Constants.AutoAlignConstants.turnKp,
      Constants.AutoAlignConstants.turnKi,
      Constants.AutoAlignConstants.turnKd);
      // Constants.AutoAlignConstants.turnConstraints);

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

    goalPose = Constants.AutoAlignConstants.goalPose.get(Constants.AutoAlignConstants.AlignPosition.MIDDLE);

    xController.setGoal(goalPose.getX());
    yController.setGoal(goalPose.getY());
    turnController.setSetpoint(m_drivetrain.getAngle() + m_limeLight.getTargetYaw() + goalPose.getRotation().getDegrees());

    xController.setTolerance(.01);
    yController.setTolerance(.01);
    turnController.setTolerance(1);

    xController.reset(m_limeLight.getTargetX());
    yController.reset(m_limeLight.getTargetY());
    turnController.reset();

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

    xSpeed = xController.calculate(m_limeLight.getTargetX());
    if (xController.atGoal()) {
      xSpeed = 0.0;
    }

    ySpeed = yController.calculate(m_limeLight.getTargetY());
    if (yController.atGoal()) {
      ySpeed = 0.0;
    }

    turnSpeed = MathUtil.clamp(turnController.calculate(m_drivetrain.getAngle()), -0.5, 0.5);

    SmartDashboard.putNumber("AutoAlignXSpeed: ", xSpeed);
    SmartDashboard.putNumber("AutoAlignYSpeed: ", ySpeed);
    SmartDashboard.putNumber("AutoAlignTurnSpeed: ", turnSpeed);
    SmartDashboard.putNumber("AutoAlignTurnGoal", turnController.getSetpoint());

    if (xController.atGoal() && yController.atGoal() && turnController.atSetpoint()) {
      System.out.println("At Goal " + Timer.getFPGATimestamp());
      end = true;
      return;
    }

    m_drivetrain.drive(xSpeed, ySpeed, turnSpeed, false, false);
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