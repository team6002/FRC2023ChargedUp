// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SUB_Elevator extends SubsystemBase {
  private final CANSparkMax m_elevatorMotor;
  private final SparkMaxPIDController m_elevatorMotorPIDController;
  private final RelativeEncoder m_elevatorEncoder;
  private SimpleMotorFeedforward m_feedForward;
  private TrapezoidProfile.Constraints m_constraints;
  private TrapezoidProfile.State m_goal;
  private TrapezoidProfile.State m_setpoint;
  private static double deltaTime = 0.02;
  private boolean m_elevatorOn = false;

    public SUB_Elevator() {
      m_elevatorMotor = new CANSparkMax(ElevatorConstants.kElevatorMotorCanID, MotorType.kBrushless);
      m_elevatorMotorPIDController = m_elevatorMotor.getPIDController();
      m_elevatorEncoder = m_elevatorMotor.getEncoder();
      m_elevatorMotor.restoreFactoryDefaults();
      m_elevatorEncoder.setPositionConversionFactor(.641);
      m_elevatorEncoder.setVelocityConversionFactor(0.641/60);
      m_elevatorMotor.setInverted(true);
      m_elevatorMotorPIDController.setP(ElevatorConstants.kElevatorP,1);
      m_elevatorMotorPIDController.setI(ElevatorConstants.kElevatorI,1);
      m_elevatorMotorPIDController.setD(ElevatorConstants.kElevatorD,1);
      // m_elevatorMotorPIDController.setFF(ElevatorConstants.kElevatorF,1);
      m_elevatorMotorPIDController.setFeedbackDevice(m_elevatorEncoder);
      m_elevatorMotor.setIdleMode(IdleMode.kCoast);
      m_elevatorMotor.setSmartCurrentLimit(50);
      m_elevatorMotorPIDController.setPositionPIDWrappingEnabled(false);
      m_elevatorMotorPIDController.setOutputRange(-1, 1, 1);
      m_elevatorMotorPIDController.setSmartMotionMaxVelocity(50, 1);
      m_elevatorMotorPIDController.setSmartMotionMaxAccel(40, 1);
      m_elevatorMotorPIDController.setSmartMotionAllowedClosedLoopError(1, 1);
      m_elevatorMotorPIDController.setSmartMotionAccelStrategy(SparkMaxPIDController.AccelStrategy.kTrapezoidal, 1);
      m_elevatorMotor.burnFlash();
      m_elevatorEncoder.setPosition(0);
          
      m_feedForward = new SimpleMotorFeedforward(1, 1.5);
      m_constraints = new TrapezoidProfile.Constraints(ElevatorConstants.kElevatorMaxVelocity, ElevatorConstants.kElevatorMaxAcceleration);
      m_setpoint = new TrapezoidProfile.State(getPosition(), 0); 
      m_goal = m_setpoint;
     
      m_elevatorOn = true;
    }
    
    public void elevatorInit(){
      m_setpoint = new TrapezoidProfile.State(getPosition(), 0); 
      m_goal = m_setpoint;
    }

    public void setReference(double p_reference){
      m_setpoint = new TrapezoidProfile.State(getPosition(),0); 
      m_goal = new TrapezoidProfile.State(p_reference, 0);
    }

    public double getPosition(){
      return m_elevatorEncoder.getPosition();
    }

    public void setElevatorOff(){
      // m_elevatorMotorPIDController.setReference(m_elevatorEncoder.getPosition(), ControlType.kSmartMotion);
    }

    public void setElevatorConstraints(double p_velocity, double p_acceleration){
      m_constraints = new TrapezoidProfile.Constraints(p_velocity, p_acceleration);
    }

    public void setElevatorReverse(){
      // m_elevatorMotor.set(-ElevatorConstants.kElevatorForward);
      m_elevatorMotor.set(-.2);
    }

  @Override
  public void periodic() {
      // updates elevator telemetry
      telemetry(); 
      if (m_elevatorOn){
        var profile = new TrapezoidProfile(m_constraints, m_goal, m_setpoint);
        m_setpoint = profile.calculate(deltaTime);
        
        m_elevatorMotorPIDController.setReference(
          m_setpoint.position, 
          CANSparkMax.ControlType.kPosition,(1)
          // m_feedForward.calculate(m_setpoint.velocity)
        );
      }
    }

  public void setPower(double p_power){
    m_elevatorMotor.set(p_power);
  }
  
  public void setElevatorOn(boolean p_state){
    m_elevatorOn = p_state;
  }

  public double getElevatorCurrent(){
    return m_elevatorMotor.getOutputCurrent();
  }
  
  public void resetElevatorPosition(){
    m_elevatorEncoder.setPosition(0);
  }
  // double m_P = 0;//elevatorConstants.kelevatorP;
  // double m_I = 0;//elevatorConstants.kelevatorI;
  // double m_D = 0;//elevatorConstants.kelevatorD;
  // double m_S = 0;//elevatorConstants.kelevatorF;
  // double m_V = 0;
  // double m_velocity = 0;
  // double m_acceleration =0;
  // double m_wantedPosition = 0;
  public void telemetry(){
    // SmartDashboard.putNumber("elevatorCurrent", getElevatorCurrent());
    SmartDashboard.putNumber("goal", m_goal.position);
    SmartDashboard.putNumber("elevator Position", m_elevatorEncoder.getPosition());
    // m_P = SmartDashboard.getNumber("P", m_P);
    // m_I = SmartDashboard.getNumber("I", m_I);
    // m_D = SmartDashboard.getNumber("D", m_D);
    // m_S = SmartDashboard.getNumber("S", m_S);
    // m_V = SmartDashboard.getNumber("V", m_V);
    // m_acceleration = SmartDashboard.getNumber("acceleration", m_acceleration);
    // m_velocity = SmartDashboard.getNumber("velocity", m_velocity);
    // m_wantedPosition = SmartDashboard.getNumber("wantedPosition", m_wantedPosition);

    // SmartDashboard.putNumber("P", m_P);
    // SmartDashboard.putNumber("I", m_I);
    // SmartDashboard.putNumber("D", m_D);
    // SmartDashboard.putNumber("S", m_S);
    // SmartDashboard.putNumber("V", m_V);
    // SmartDashboard.putNumber("acceleration", m_acceleration);
    // SmartDashboard.putNumber("velocity", m_velocity);
    // SmartDashboard.putNumber("wantedPosition", m_wantedPosition);
   
    // m_elevatorMotorPIDController.setP(m_P,1);
    // m_elevatorMotorPIDController.setI(m_I,1);
    // m_elevatorMotorPIDController.setD(m_D,1);
    // m_feedForward = new SimpleMotorFeedforward(m_S, m_V);
    // m_constraints = new TrapezoidProfile.Constraints(m_velocity, m_acceleration);
    // m_goal = new TrapezoidProfile.State(m_wantedPosition, 0);
    // m_elevatorMotorPIDController.setReference(m_wantedPosition, ControlType.kSmartMotion, 1);

    // SmartDashboard.putNumber("output", m_elevatorMotor.getAppliedOutput());
    // SmartDashboard.putNumber("velocity", m_elevatorEncoder.getVelocity());
    // SmartDashboard.putNumber("wantedspeed", m_elevatorMotor.get());
   }
}
