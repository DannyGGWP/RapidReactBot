// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class DriveTrain extends SubsystemBase {

  private final WPI_TalonFX m_motorFR;
  private final WPI_TalonFX m_motorFL;
  private final WPI_TalonFX m_motorBR;
  private final WPI_TalonFX m_motorBL;

  private final MotorControllerGroup m_controllerGroupL;
  private final MotorControllerGroup m_controllerGroupR;

  private final DifferentialDrive m_differentialDrive;

  public AHRS m_gyro;

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    m_motorFR = new WPI_TalonFX(0);
    m_motorFL = new WPI_TalonFX(1);
    m_motorBR = new WPI_TalonFX(2);
    m_motorBL = new WPI_TalonFX(3);

    m_controllerGroupL = new MotorControllerGroup(m_motorFL, m_motorBL);
    m_controllerGroupR = new MotorControllerGroup(m_motorFR, m_motorBR);

    m_differentialDrive = new DifferentialDrive(m_controllerGroupL, m_controllerGroupR);

    m_gyro = new AHRS(Port.kMXP);
  }

  public double getEncPos() {
    int encFL = -m_motorFL.getSelectedSensorPosition(0);
    int encFR = -m_motorFR.getSelectedSensorPosition(0);
    int encBL = -m_motorBL.getSelectedSensorPosition(0);
    int encBR = -m_motorBR.getSelectedSensorPosition(0);
    SmartDashboard.putNumber("Front Left Enc", encFL);
    SmartDashboard.putNumber("Front Right Enc", encFR);
    SmartDashboard.putNumber("Back Left Enc", encBL);
    SmartDashboard.putNumber("Back Right Enc", encBR);
    
    double position = (encFL + encFR + encBL + encBR) / 4;
    return position;
  }

  public double getHeading() {
    return Math.IEEEremainder(m_gyro,getAngle(), 360);
  }

  public void resetEncoders() {
    m_motorFL.setSelectedSensorPosition(0);
    m_motorFR.setSelectedSensorPosition(0);
    m_motorBL.setSelectedSensorPosition(0);
    m_motorBR.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
