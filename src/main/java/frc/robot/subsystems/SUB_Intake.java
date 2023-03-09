// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SUB_Intake extends SubsystemBase {
    private final CANSparkMax m_intakeMotor;
    private final SparkMaxPIDController m_intakeMotorPIDController;
    private final DigitalInput m_sensor;
    private boolean m_cubeDetected = false;
  /** Creates a new SUB_Intake. */
  public SUB_Intake() {
    m_intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorCanID, MotorType.kBrushless);
    m_intakeMotorPIDController = m_intakeMotor.getPIDController();
    m_sensor = new DigitalInput(1);
    m_intakeMotor.restoreFactoryDefaults();
    m_intakeMotor.setIdleMode(IdleMode.kBrake);
    m_cubeDetected = false;
    m_intakeMotor.burnFlash();
  }

  public void setIntakeForward(){
    // m_intakeMotorPIDController.setReference(IntakeConstants.kIntakeForward, CANSparkMax.ControlType.kVelocity);
    m_intakeMotor.set(IntakeConstants.kIntakeForwardPower);
  }

  public void setIntakeOff(){
    m_intakeMotorPIDController.setReference(0, ControlType.kVelocity);
  }

  public void setIntakeReverse(){
    // m_intakeMotorPIDController.setReference(-IntakeConstants.kIntakeForward, CANSparkMax.ControlType.kVelocity);
    m_intakeMotor.set(-IntakeConstants.kIntakeForwardPower);
  }

  public boolean getSensor(){
    return m_sensor.get();
  }

  public void Off(){}

  public void setIntakeCurrent(){
    m_intakeMotor.setSmartCurrentLimit(50);
  }

  public void setHoldCurrent(){
    m_intakeMotor.setSmartCurrentLimit(5);
    // System.out.println(m_intakeMotor.getOutputCurrent());
  }

  public void setPower(double speed){
    m_intakeMotor.set(speed);
  }

  public double getCurrent(){
    return m_intakeMotor.getOutputCurrent();
  }
  //Note make sure to remove this when we get our commands intruptable
  public void setCubeDetected(boolean p_detected){
    m_cubeDetected = p_detected;
  }
  public boolean getCubeDetected(){
    return m_cubeDetected;
  }
  
  @Override
  public void periodic(){
  }
}
