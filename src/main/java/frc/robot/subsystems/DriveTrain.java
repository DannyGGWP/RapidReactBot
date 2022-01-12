// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
