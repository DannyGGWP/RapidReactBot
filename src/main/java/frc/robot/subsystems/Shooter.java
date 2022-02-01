// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/** Add your docs here. */
public class Shooter extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

private CANSparkMax shooterMotor; 
private SparkMaxPIDController m_pPidController;
public double kP,kI,kD,kIZ, kFF,kMaxOutput, kMinOutput, kMaxRPM;

public Shooter(){

  shooterMotor = new CANSparkMax(Constants.shooterSpark, MotorType.kBrushless);
  m_pPidController = shooterMotor.getPIDController();
  kP = 5e-5;
  kI = 3e-7;
  kD = 0.008;
  kIZ = 0;
  kFF = 0;
  kMaxOutput = 1;
  kMinOutput = 1;
  kMaxRPM = 5700; 
  m_pPidController.setP(kP);
  m_pPidController.setI(kI);
  m_pPidController.setD(kD);
  m_pPidController.setIZone(kIZ);
  m_pPidController.setFF(kFF);
  m_pPidController.setOutputRange(kMinOutput, kMaxOutput);
}
  @Override 
  public void periodic(){}
@Override
public void simulationPeriodic() {
  // TODO Auto-generated method stub
  super.simulationPeriodic();

}
}
