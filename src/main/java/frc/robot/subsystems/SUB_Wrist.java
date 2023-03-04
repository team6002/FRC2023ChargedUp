// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import frc.robot.Constants.WristConstants;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SUB_Wrist extends SubsystemBase {

    private final CANSparkMax m_wristMotor;
    private final SparkMaxPIDController m_wristMotorPIDController;
    private final AbsoluteEncoder m_wristAbsoluteEncoder;
    private final RelativeEncoder m_wristEncoder;
    double m_wantedPosition;
    double m_tolerance = 6;

    private SimpleMotorFeedforward m_feedForward;
    private TrapezoidProfile.Constraints m_constraints;
    private TrapezoidProfile.State m_goal;
    private TrapezoidProfile.State m_setpoint;
    private static double deltaTime = 0.02;
    private boolean m_wristOn = false;
    
    public SUB_Wrist() {
      m_wristMotor = new CANSparkMax(WristConstants.kWristMotorCanID, MotorType.kBrushless);
      m_wristMotorPIDController = m_wristMotor.getPIDController();
      m_wristMotor.restoreFactoryDefaults();
      m_wristAbsoluteEncoder = m_wristMotor.getAbsoluteEncoder(Type.kDutyCycle);
      m_wristEncoder = m_wristMotor.getEncoder();

      m_wristMotor.setInverted(false);
      m_wristAbsoluteEncoder.setPositionConversionFactor(360);
      m_wristAbsoluteEncoder.setVelocityConversionFactor(6);
      m_wristAbsoluteEncoder.setInverted(false);

      m_wristEncoder.setPositionConversionFactor(2.43);
      m_wristEncoder.setVelocityConversionFactor(.04);
      m_wristEncoder.setPosition(m_wristAbsoluteEncoder.getPosition());

      m_wristMotor.setSmartCurrentLimit(30);

      // m_wristMotor.setSoftLimit(null, 80);
      m_wristMotorPIDController.setP(WristConstants.kWristP,1);
      m_wristMotorPIDController.setI(WristConstants.kWristI,1);
      m_wristMotorPIDController.setD(WristConstants.kWristI,1);
      m_wristMotorPIDController.setFF(WristConstants.kWristF,1);
      m_wristMotorPIDController.setFeedbackDevice(m_wristEncoder);
      m_wristMotorPIDController.setPositionPIDWrappingEnabled(false);
      m_wristMotorPIDController.setOutputRange(WristConstants.kWristMinOutput, WristConstants.kWristMaxOutput, 1);
      m_wristMotorPIDController.setSmartMotionMaxVelocity(10, 1);
      m_wristMotorPIDController.setSmartMotionMinOutputVelocity(-0, 1);
      m_wristMotorPIDController.setSmartMotionMaxAccel(5, 1);
      m_wristMotorPIDController.setSmartMotionAllowedClosedLoopError(1, 1);
      m_wristMotorPIDController.setSmartMotionAccelStrategy(SparkMaxPIDController.AccelStrategy.kTrapezoidal, 1);
    
      // SmartDashboard.putNumber("Wrist P", WristConstants.kWristP);
      // SmartDashboard.putNumber("Wrist I", WristConstants.kWristI);
      // SmartDashboard.putNumber("Wrist D", WristConstants.kWristD);
      // SmartDashboard.putNumber("Wrist F", WristConstants.kWristF);

      m_feedForward = new SimpleMotorFeedforward(1, 1.5);
      m_constraints =
      new TrapezoidProfile.Constraints(WristConstants.kWristMaxVelocity, WristConstants.kWristMaxAcceleration);
      m_setpoint = new TrapezoidProfile.State(getWristPosition(), 0); 
      m_goal = m_setpoint;
      
      m_wristOn = false;
      m_wristMotor.burnFlash();
    }

    public void wristInit(){
      m_setpoint = new TrapezoidProfile.State(getWristPosition(), 0); 
      m_goal = m_setpoint;
    }
    
    public void setReference(double p_reference){
      m_setpoint = new TrapezoidProfile.State(getWristPosition(), 0);
      m_goal = new TrapezoidProfile.State(p_reference, 0);
      m_wantedPosition = p_reference;
    }

    public double getWristPosition(){
      return m_wristEncoder.getPosition();
    }

    public double getWristAbsolutePosition(){
      return m_wristEncoder.getPosition();
    }

    public void setWristOn(boolean p_state){
      m_wristOn = p_state;
    }

    public void setWristConstraints(double p_velocity, double p_acceleration){
      m_constraints = new TrapezoidProfile.Constraints(p_velocity, p_acceleration);
    }

    @Override
    public void periodic() {
      telemetry();
        var profile = new TrapezoidProfile(m_constraints, m_goal, m_setpoint);
        m_setpoint = profile.calculate(deltaTime);

        m_wristMotorPIDController.setReference(
          m_setpoint.position, 
          CANSparkMax.ControlType.kPosition,(1)
        );

    }

    public double getWristVelocity(){
      return m_wristEncoder.getVelocity();
    }
    public boolean checkPosition(){
      if(Math.abs(getWristPosition() - m_wantedPosition) < m_tolerance){
        return true;
      }else{
        return false;
      }
    }
    // double m_P,m_I,m_D,m_F, m_minOutput, m_maxOutput;
    
    // double m_P = 0;//elbowConstants.kelbowP;
    // double m_I = 0;//elbowConstants.kelbowI;
    // double m_D = 0;//elbowConstants.kelbowD;
    // double m_S = 0;//elbowConstants.kelbowF;
    // double m_V = 0;
    // double m_acceleration = 0;
    // double m_velocity = 0;
    public void telemetry(){
      SmartDashboard.putNumber("wrist position", m_wristEncoder.getPosition());
      SmartDashboard.putNumber("Wrist absolute position", m_wristAbsoluteEncoder.getPosition());
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
     
      // m_wristMotorPIDController.setP(m_P,1);
      // m_wristMotorPIDController.setI(m_I,1);
      // m_wristMotorPIDController.setD(m_D,1);
      // m_feedForward = new SimpleMotorFeedforward(m_S, m_V);
      // m_constraints = new TrapezoidProfile.Constraints(m_velocity, m_acceleration);
      // m_goal = new TrapezoidProfile.State(m_wantedPosition, 0);
      // // m_wristMotorPIDController.setReference(m_wantedPosition, ControlType.kSmartMotion, 1);
  
      // SmartDashboard.putNumber("output", m_wristMotor.getAppliedOutput());
    }
}
