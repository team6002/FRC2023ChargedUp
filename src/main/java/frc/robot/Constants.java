// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(18.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(22.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 14;
    public static final int kRearLeftDrivingCanId = 15;
    public static final int kFrontRightDrivingCanId = 5;
    public static final int kRearRightDrivingCanId = 4;

    public static final int kFrontLeftTurningCanId = 13;
    public static final int kRearLeftTurningCanId = 16;
    public static final int kFrontRightTurningCanId = 6;
    public static final int kRearRightTurningCanId = 3;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = .00008;// 0.01;
    public static final double kDrivingI = 0.0;
    public static final double kDrivingD = 0.0;
    public static final double kDrivingFF = 0.167; //.203;
    public static final double kDrivingS = 0.1;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1.5; //.7;
    public static final double kTurningI = 0.0;
    public static final double kTurningD = 0.0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.15;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 4;
    public static final double kMaxAccelerationMetersPerSecondSquared = 4;
    public static final double kChargeStationSpeed = 1.75;
    public static final double kChargeStationAcceleration = 1.75;

    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1.5;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1.5;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class IntakeConstants{
    public static final int kIntakeMotorCanID = 12;
    //intake powers
    public static final double kIntakeForwardPower = .7;
    public static final double kIntakeHoldPower = .07;
    public static final double kIntakeDropCone = -0.7;
    public static final double kIntakeDropCube = 0.3;// the cubes are lighter than cones
    public static final double kIntakeShoot = 1;
    //intake currents
    public static final double kIntakeCurrent = 35;
    public static final double kHoldCurrent = 5;

    public static final double kIntakeConeDetectedCurrent = 30;

  }

  public static final class ElevatorConstants{
    public static final int kElevatorMotorCanID = 17;
    public static final double kElevatorP = 0.5;
    public static final double kElevatorI = 0;
    public static final double kElevatorD = 0.;
    public static final double kElevatorF = 0.0;
    public static final double kElevatorOffset = -.487;

    public static final double kElevatorMax = 48;
    public static final double kElevatorShelf = 44.5;
    public static final double kElevatorFirstLevel = 21;// for placeing forwards
    public static final double kElevatorSecondLevel = 26;  
    public static final double kElevatorThirdLevel = 43;
    public static final double kElevatorHome = 1;
    public static final double kElevatorStow = 1;
    public static final double kElevatorGround = 1;
    public static final double kElevatorPrep = 20;//Position for safely moving elbow to score
    public static final double kElevatorSafety = 15;//maximum value to check if moving wrist endagers robot
    public static final double kElevatorTolerance = 3;
    
    public static final double kElevatorHomeCurrent = 10;

    public static final double kElevatorMaxVelocity = 60;
    public static final double kElevatorMaxAcceleration = 60;
    public static final double kElevatorHomeAcceleration = 3;
    public static final double kElevatorHomeVelocity = 3;
    
  }

  public static final class ElbowConstants{
    public static final int kElbowMotorCanID = 11;
    public static final double kElbowP = .0075;//.0075
    public static final double kElbowI = 0;
    public static final double kElbowD = 0.0;
    public static final double kElbowF = 0.0;
    public static final double kElbowS = 0.04;
    public static final double kElbowMaxVelocity = 600;
    public static final double kElbowMaxAcceleration = 600;
    public static final double kElbowMinOutput = -1;
    public static final double kElbowMaxOutput = 1;

    public static final double kElbowStowBackwards = 220;//when wrist faces backwards
    public static final double kElbowStowForwards = 212;// when wrist faces forwards
    public static final double kElbowSaftey = 190;//maximum safe value for rotating wrist
    public static final double kElbowLift = 190;
    public static final double kElbowPlaceBack = 110; // for placing on ground
    public static final double kElbowUp = 175;// about 90 degrees up
    public static final double kElbowForwards = 270;// stright forwards
    public static final double kElbowPrepareDrop = 250;// Not quite all the way forwards, allows cones to clear the pole
    public static final double kElbowBackwards = 90;// straight back

    public static final double kElbowTolerance = 1;
  }
  
  

  public static final class WristConstants{
    public static final int kWristMotorCanID = 10;
    public static final double kWristP = 0.011;
    public static final double kWristI = 0;
    public static final double kWristD = 0.0;
    public static final double kWristF = 0.0;
    public static final double kWristMaxOutput = 1;
    public static final double kWristMinOutput = -1;

    public static final double kWristGround = 90;//back
    public static final double kWristShelf = 270;//front
    
    public static final double kWristTolerance = 5;
  
    public static final double kWristMaxVelocity = 800;
    public static final double kWristMaxAcceleration = 800;
    public static final double kWristTestVelocity = 1;
    public static final double kWristTestAcceleration = 1;
  }

  public static final class GlobalConstants{
    public static final int kPickBackGroundMode = 0;
    public static final int kPickForwardsShelfMode = 1;
    public static final int kPickForwardsGroundMode = 2;

    public static final int kStowGroundMode = 0;
    public static final int kStowShelfMode = 1;

    public static final int kElevator1stLevel = 1;
    public static final int kElevator2ndLevel = 2;
    public static final int kElevator3rdLevel = 3;

    public static final boolean kConeMode = true;
    public static final boolean kCubeMode = false;
  }


}