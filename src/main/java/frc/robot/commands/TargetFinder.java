// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class TargetFinder extends CommandBase {
  private DriveTrain m_dDriveTrain; 
  private boolean m_hasTarget; 
  private boolean m_atTarget; 
  /** Creates a new TargetFinder. */
  public TargetFinder(DriveTrain drive) {
    m_dDriveTrain = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_dDriveTrain);


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_hasTarget = false; 
    m_atTarget = false; 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0); 
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0); 
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0); 
    double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    
    if (tv < 1.0)
    {
      m_hasTarget = false; 
      return; 
    }
    m_hasTarget = true; 

    double turnValue = tx * Constants.LimeLight.kSteerP; 
    double driveValue = (Constants.LimeLight.kDesiredTarget - ta) * Constants.LimeLight.kDriveP; 
    if (driveValue < Constants.LimeLight.kMinSpeed)
      m_atTarget = true; 
    if (driveValue > Constants.LimeLight.kMaxDrive)
      driveValue = Constants.LimeLight.kMaxDrive; 
    m_dDriveTrain.drive(driveValue, turnValue);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_dDriveTrain.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!m_hasTarget)
    {
      return true; 
    }
    else if (m_hasTarget && m_atTarget)
    {
      return true; 
    }
    return false; 
  }
}
