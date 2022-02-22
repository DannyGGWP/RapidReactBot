// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class Hanger extends SubsystemBase {

  /** Creates a new ExampleSubsystem. */
  private CANSparkMax m_hangerMotor;
  private boolean m_enabled;
  public Hanger() {
    m_hangerMotor = new CANSparkMax(Constants.kHangerSpark, MotorType.kBrushless);
    m_hangerMotor.setIdleMode(IdleMode.kBrake);
    m_enabled = false;
  }
  public void enableHanger() {
    m_enabled = true;
  }
  public void disableHanger() {
    m_enabled = false;
    stopHang();
  }
  public void raiseHanger(){
    if (!m_enabled) {
      return;
    }
    m_hangerMotor.set(-0.5);
  }
  public void lowerHanger(){
    if (!m_enabled) {
      return;
    }
    m_hangerMotor.set(0.25);
  }
  public void stopHang(){
    m_hangerMotor.set(0.0);
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
