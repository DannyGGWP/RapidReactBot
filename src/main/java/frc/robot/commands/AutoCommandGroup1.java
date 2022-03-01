// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoCommandGroup1 extends SequentialCommandGroup {
  private DriveTrain m_driveTrain;
  private double m_encPos;
  /** Creates a new AutoCommandGroup1. */
  public AutoCommandGroup1(Grabber grabber, Shooter shooter, DriveTrain drive) {
    m_driveTrain = drive;
    m_driveTrain.resetHeading();
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelDeadlineGroup(
        new ParallelCommandGroup(
          new ScheduleCommand(
            new AutoGrabbyCommand(grabber, shooter)
          ),
          new BallAtGrabberCommand(grabber)
        ),
        new SequentialCommandGroup(
          new WaitCommand(1),
          new TargetFinder(m_driveTrain)
        )
      ) ,
      new AutoDriveCommand(m_driveTrain, -50000, 0.5) , // 30000 less that travelled ~150000
      new ParallelRaceGroup(
        new WaitCommand(1),
        new TurnToAngle(
          180,
          m_driveTrain,
          0.0113,
          0.0000,
          0.0025,
          0
        )
      ),
      new ShootyCommand(shooter, grabber)
    );
  }
}
