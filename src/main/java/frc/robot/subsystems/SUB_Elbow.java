// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import frc.robot.Constants.ElbowConstants;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SUB_Elbow extends SubsystemBase {

    private final CANSparkMax m_elbowMotor;
    private final SparkMaxPIDController m_elbowMotorPIDController;
    private final AbsoluteEncoder m_elbowAbsoluteEncoder;
    private final RelativeEncoder m_elbowEncoder;
    double m_wantedPosition;
    private SimpleMotorFeedforward m_feedForward;
    private TrapezoidProfile.Constraints m_constraints;
    private TrapezoidProfile.State m_goal;
    private TrapezoidProfile.State m_setpoint;
    private static double deltaTime = 0.02;
    private boolean m_elbowOn = false;

    public SUB_Elbow() {
        m_elbowMotor = new CANSparkMax(ElbowConstants.kElbowMotorCanID, MotorType.kBrushless);
        m_elbowMotorPIDController = m_elbowMotor.getPIDController();
        m_elbowMotor.restoreFactoryDefaults();
        m_elbowAbsoluteEncoder = m_elbowMotor.getAbsoluteEncoder(Type.kDutyCycle);
        m_elbowEncoder = m_elbowMotor.getEncoder();
        
        m_elbowAbsoluteEncoder.setPositionConversionFactor(360);
        m_elbowAbsoluteEncoder.setVelocityConversionFactor(6);
        m_elbowAbsoluteEncoder.setInverted(true);
        
        m_elbowEncoder.setPositionConversionFactor(5.128);
        m_elbowEncoder.setVelocityConversionFactor(5.128/60);
      

        m_elbowMotorPIDController.setP(ElbowConstants.kElbowP,1);
        m_elbowMotorPIDController.setI(ElbowConstants.kElbowI,1);
        m_elbowMotorPIDController.setD(ElbowConstants.kElbowD,1);
        m_elbowMotorPIDController.setFF(0,1);
        m_elbowMotorPIDController.setFeedbackDevice(m_elbowEncoder);

        m_elbowMotor.setIdleMode(IdleMode.kCoast);
        m_elbowMotor.setSmartCurrentLimit(50);

        m_elbowMotorPIDController.setPositionPIDWrappingEnabled(false);
        m_elbowMotorPIDController.setOutputRange(ElbowConstants.kElbowMinOutput, ElbowConstants.kElbowMaxOutput, 1);
        m_elbowMotorPIDController.setSmartMotionMaxVelocity(10, 1);
        m_elbowMotorPIDController.setSmartMotionMinOutputVelocity(-0, 1);
        m_elbowMotorPIDController.setSmartMotionMaxAccel(10, 1);
        m_elbowMotorPIDController.setSmartMotionAllowedClosedLoopError(1, 1);
        m_elbowMotorPIDController.setSmartMotionAccelStrategy(SparkMaxPIDController.AccelStrategy.kTrapezoidal, 1);
   
        // SmartDashboard.putNumber("Elbow P", ElbowConstants.kElbowP);
        // SmartDashboard.putNumber("Elbow I", ElbowConstants.kElbowI);
        // SmartDashboard.putNumber("Elbow D", ElbowConstants.kElbowD);
        // SmartDashboard.putNumber("Elbow F", ElbowConstants.kElbowF);

        m_feedForward = new SimpleMotorFeedforward(ElbowConstants.kElbowS, ElbowConstants.kElbowV);
        m_constraints = new TrapezoidProfile.Constraints(ElbowConstants.kElbowMaxVelocity, ElbowConstants.kElbowMaxAcceleration);
        m_setpoint = new TrapezoidProfile.State(getPosition(), 0); 
        m_goal = m_setpoint;
        m_elbowOn = false;

        m_elbowEncoder.setPosition(getAbsolutePosition());
        
        m_elbowMotor.burnFlash();
    }

    public void elbowInit(){
        m_setpoint = new TrapezoidProfile.State(getPosition(), 0); 
        m_goal = m_setpoint;  
    }
    public void setReference(double p_reference){
        m_wantedPosition = p_reference;
        m_setpoint = new TrapezoidProfile.State(getPosition(), 0);
        m_goal = new TrapezoidProfile.State(p_reference, 0);
    }

    public double getPosition(){
        return m_elbowEncoder.getPosition();
    }

    public double getAbsolutePosition(){
        return m_elbowAbsoluteEncoder.getPosition();
    }

    public double getElbowWantedPosition(){
        return m_wantedPosition;
    }

    public void setElbowConstraints(double p_velocity, double p_acceleration){
        m_constraints = new TrapezoidProfile.Constraints(p_velocity, p_acceleration);
    }
    public void setElbowOn(Boolean p_state){
        m_elbowOn = p_state;
    }

    @Override
    public void periodic() {
        telemetry();
        var profile = new TrapezoidProfile(m_constraints, m_goal, m_setpoint);
        
        m_setpoint = profile.calculate(deltaTime);

        m_elbowMotorPIDController.setReference(
            m_setpoint.position,
            CANSparkMax.ControlType.kPosition, (1),
            m_feedForward.calculate(m_setpoint.velocity)
        );
    }

    
    // double m_P = 0;//elbowConstants.kelbowP;
    // double m_I = 0;//elbowConstants.kelbowI;
    // double m_D = 0;//elbowConstants.kelbowD;
    // double m_S = 0;//elbowConstants.kelbowF;
    // double m_V = 0;
    // double m_acceleration = 0;
    // double m_velocity = 0;
    public void telemetry(){

      SmartDashboard.putNumber("elbow position", m_elbowEncoder.getPosition());
      SmartDashboard.putNumber("absoluteElbow postion", m_elbowAbsoluteEncoder.getPosition());
    //   m_P = SmartDashboard.getNumber("P", m_P);
    //   m_I = SmartDashboard.getNumber("I", m_I);
    //   m_D = SmartDashboard.getNumber("D", m_D);
    //   m_S = SmartDashboard.getNumber("S", m_S);
    //   m_V = SmartDashboard.getNumber("V", m_V);
    //   m_acceleration = SmartDashboard.getNumber("acceleration", m_acceleration);
    //   m_velocity = SmartDashboard.getNumber("velocity", m_velocity);
    //   m_wantedPosition = SmartDashboard.getNumber("wantedPosition", m_wantedPosition);
  
    //   SmartDashboard.putNumber("P", m_P);
    //   SmartDashboard.putNumber("I", m_I);
    //   SmartDashboard.putNumber("D", m_D);
    //   SmartDashboard.putNumber("S", m_S);
    //   SmartDashboard.putNumber("V", m_V);
    //   SmartDashboard.putNumber("acceleration", m_acceleration);
    //   SmartDashboard.putNumber("velocity", m_velocity);
    //   SmartDashboard.putNumber("wantedPosition", m_wantedPosition);
     
    //   m_elbowMotorPIDController.setP(m_P,1);
    //   m_elbowMotorPIDController.setI(m_I,1);
    //   m_elbowMotorPIDController.setD(m_D,1);
    //   m_feedForward = new SimpleMotorFeedforward(m_S, m_V);
    //   m_constraints = new TrapezoidProfile.Constraints(m_velocity, m_acceleration);
    //   m_goal = new TrapezoidProfile.State(m_wantedPosition, 0);
      
    //   SmartDashboard.putNumber("output", m_elbowMotor.getAppliedOutput());//   SmartDashboard.putNumber("elbowSetpoint", m_wantedPosition);
    }
}
