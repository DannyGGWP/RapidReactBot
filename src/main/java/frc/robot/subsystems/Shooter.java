// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;




/** Add your docs here. */
public class Shooter extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

private CANSparkMax shooterMotor; 
private SparkMaxPIDController m_pPidController;
public double kP,kI,kD,kIZ, kFF,kMaxOutput, kMinOutput, kMaxRPM;
private Solenoid m_shootySolenoid;
private DigitalInput m_shooterSensor;
private boolean m_isRunning;

public Shooter(){

  shooterMotor = new CANSparkMax(Constants.kshooterSpark, MotorType.kBrushless);
  shooterMotor.setInverted(false);
  m_pPidController = shooterMotor.getPIDController();
  kP = 5e-5;
  kI = 3e-7;
  kD = 0.008;
  kIZ = 0;
  kFF = 0;
  kMaxOutput = 1;
  kMinOutput = -1;
  kMaxRPM = 5700; 
  m_pPidController.setP(kP);
  m_pPidController.setI(kI);
  m_pPidController.setD(kD);
  m_pPidController.setIZone(kIZ);
  m_pPidController.setFF(kFF);
  m_pPidController.setOutputRange(kMinOutput, kMaxOutput);
  m_shootySolenoid = new Solenoid(Constants.kPCM, PneumaticsModuleType.CTREPCM, Constants.kShootySolenoidIndex);
  m_shooterSensor = new DigitalInput(Constants.kSenseyShooty);
}
  @Override 
  public void periodic(){
    SmartDashboard.putNumber("Wheel Speed", shooterMotor.getEncoder().getVelocity()); 
    SmartDashboard.putNumber("Wheel Motor", shooterMotor.get());

    idleSpin();
  }
  @Override
  public void simulationPeriodic() {
    // TODO Auto-generated method stub
    super.simulationPeriodic();
  }
  public void idleSpin(){
    if (!m_isRunning) {
      if(isBallReady()) {
         shooterMotor.set(Constants.kIdleSpeed);
        //m_pPidController.setReference(Constants.kIdleSpeed, ControlType.kVelocity);
      } else {
        offWheel();
      }
    }
  }

  public void onWheel(){
    m_pPidController.setReference(Constants.kSetPoint, ControlType.kVelocity);
    m_isRunning = true;
    //shooterMotor.set(0.1); 
  }

  public void offWheel(){
    shooterMotor.stopMotor();
    m_isRunning = false;
  }

  public double wheelSpin(){
    // TODO figure out methods
    return shooterMotor.getEncoder().getVelocity();
  }

  public void raiseTOT(boolean raise){
    m_shootySolenoid.set(raise);
  }

  public boolean isBallReady() {
    return m_shooterSensor.get();
  }
  public void reverse(){
    m_pPidController.setReference(Constants.kreverseSetPoint, ControlType.kVelocity);
    m_isRunning = true;
    //shooterMotor.set(-0.1);
  }
}
