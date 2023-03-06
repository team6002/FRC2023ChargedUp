package frc.robot.auto;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import java.util.List;

import javax.swing.plaf.TreeUI;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SUB_Drivetrain;
/** Add your docs here. */

public class AUTO_Trajectories {
    public Trajectory OverChargeStationTrajectory;
    public Trajectory BackOnChargeStationTrajectory;
    public Trajectory OverChargeStationTrajectory2;
    public Trajectory CubeRunTrajectory1;
    public Trajectory CubePlaceTrajectory1;
    public Trajectory CubePlaceTrajectory2;
    public Trajectory CubeRunTrajectory2;
    Trajectory FirstRedBall = new Trajectory();
    private SUB_Drivetrain m_drivetrain;

    public AUTO_Trajectories(SUB_Drivetrain drivetrain){
        m_drivetrain = drivetrain;

        TrajectoryConfig ChargeStationConfig =
            new TrajectoryConfig(
                AutoConstants.kChargeStationSpeed,
                AutoConstants.kChargeStationAcceleration)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics);

        TrajectoryConfig ChargeStationConfigReversesd =
            new TrajectoryConfig(
                AutoConstants.kChargeStationSpeed,
                AutoConstants.kChargeStationAcceleration)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics)
                .setReversed(true);

        TrajectoryConfig config =
            new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics);

        TrajectoryConfig configReversed =
            new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics)
                .setReversed(true);

        
        TrajectoryConfig configHalfSpeed =
        new TrajectoryConfig(
            AutoConstants.kHalfSpeed,
            AutoConstants.kHalfAcceleration)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);
    
        TrajectoryConfig configHalfSpeedReversed =
        new TrajectoryConfig(
            AutoConstants.kHalfSpeed,
            AutoConstants.kHalfAcceleration)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics)
            .setReversed(true);    
                    
        //Rotation2d uses RADIANS NOT DEGREES!
        //Use Rotation2d.fromDegrees(desiredDegree) instead
      
        OverChargeStationTrajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(),
            new Pose2d(Units.inchesToMeters(-120), 0, new Rotation2d(0)),
            ChargeStationConfigReversesd);
        OverChargeStationTrajectory2 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(-120), 0, new Rotation2d(0)),
            List.of(),
            new Pose2d(Units.inchesToMeters(-212), 0, new Rotation2d(0)),
            configReversed);

        BackOnChargeStationTrajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(-212), 0, new Rotation2d(Units.degreesToRadians(180))),
            List.of(),
            new Pose2d(Units.inchesToMeters(-86), 0, new Rotation2d(Units.degreesToRadians(180))),
            ChargeStationConfigReversesd); //-68 was twisted devil
        
        CubeRunTrajectory1 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(0), 0, new Rotation2d(0)),
            List.of(),
            new Pose2d(Units.inchesToMeters(-200), Units.inchesToMeters(-20), new Rotation2d(0)),
            configReversed);
    
        CubePlaceTrajectory1 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(-200), Units.inchesToMeters(-20), new Rotation2d(0)),
            List.of(new Translation2d(Units.inchesToMeters(-120), Units.inchesToMeters(-12))),
            new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(-12), new Rotation2d(0)),
            config);

        CubeRunTrajectory2 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(-12), new Rotation2d(0)),
            List.of(new Translation2d(Units.inchesToMeters(-130), Units.inchesToMeters(0))),
            new Pose2d(Units.inchesToMeters(-215), Units.inchesToMeters(-56), new Rotation2d(Units.degreesToRadians(90))),
            configReversed);

        CubePlaceTrajectory2 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(-215), Units.inchesToMeters(-56), new Rotation2d(Units.degreesToRadians(90))),
            List.of(new Translation2d(Units.inchesToMeters(-130), Units.inchesToMeters(0))),
            new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(-25), new Rotation2d(Units.degreesToRadians(0))),
            config);
    }
  
    public Command driveTrajectory(Trajectory trajectory) {
        var thetaController = new ProfiledPIDController(AutoConstants.kPThetaController,
            0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(trajectory,
            m_drivetrain::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,
            new PIDController(AutoConstants.kPXController, 0, 0), // Position controllers
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_drivetrain::setModuleStates,
            m_drivetrain);
        
        // Run path following command, then stop at the end.
        return swerveControllerCommand.andThen(() -> m_drivetrain.drive(0.0 ,0.0 ,0.0, true, false));
    }
}
