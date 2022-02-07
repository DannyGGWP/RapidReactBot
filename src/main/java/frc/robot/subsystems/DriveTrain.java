// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class DriveTrain extends SubsystemBase {

  private final WPI_TalonFX m_motorFR;
  private final WPI_TalonFX m_motorFL;
  private final WPI_TalonFX m_motorBR;
  private final WPI_TalonFX m_motorBL;

  private final MotorControllerGroup m_controllerGroupL;
  private final MotorControllerGroup m_controllerGroupR;

  private final DifferentialDrive m_differentialDrive;

  public AHRS m_gyro;

  private final DifferentialDriveOdometry m_odometry; 

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    m_motorFL = new WPI_TalonFX(Constants.kMotorFL);
    m_motorBL = new WPI_TalonFX(Constants.kMotorBL);
    m_motorFR = new WPI_TalonFX(Constants.kMotorFR);
    m_motorBR = new WPI_TalonFX(Constants.kMotorBR);

    m_controllerGroupL = new MotorControllerGroup(m_motorFL, m_motorBL);
    m_controllerGroupR = new MotorControllerGroup(m_motorFR, m_motorBR);

    m_differentialDrive = new DifferentialDrive(m_controllerGroupL, m_controllerGroupR);
    m_differentialDrive.setDeadband(0.05);

    m_gyro = new AHRS(Port.kMXP);
    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
  }

  public double getEncPos() {
    double encFL = -m_motorFL.getSelectedSensorPosition(0);
    double encFR = m_motorFR.getSelectedSensorPosition(0);
    double encBL = -m_motorBL.getSelectedSensorPosition(0);
    double encBR = m_motorBR.getSelectedSensorPosition(0);
    SmartDashboard.putNumber("Front Left Enc", encFL);
    SmartDashboard.putNumber("Front Right Enc", encFR);
    SmartDashboard.putNumber("Back Left Enc", encBL);
    SmartDashboard.putNumber("Back Right Enc", encBR);
    
    double position = (encFL + encFR + encBL + encBR) / 4;
    return position;
  }

  public double getHeading() {
    return Math.IEEEremainder(m_gyro.getAngle(), 360);
  }

  public void resetEncoders() {
    m_motorFL.setSelectedSensorPosition(0);
    m_motorFR.setSelectedSensorPosition(0);
    m_motorBL.setSelectedSensorPosition(0);
    m_motorBR.setSelectedSensorPosition(0);
  }

  public void drive(double x, double y) {
    m_differentialDrive.arcadeDrive(Math.abs(x) < Constants.kDriveThreshold ? x : Constants.kDriveReduction, Math.abs(y) < Constants.kDriveThreshold ? y : Constants.kDriveReduction);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
