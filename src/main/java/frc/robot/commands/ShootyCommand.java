// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Shooter;

public class ShootyCommand extends CommandBase {

  private enum states {
    START,
    RAMPWHEEL,
    RAISETOT,
    LOWERTOT,
    RELEASEBALL,
    END
  }

  private states currentState;
  private double timestamp;
  private Shooter m_shooter;
  private Grabber m_grabber;

  /** Creates a new ShootyCommand. */
  public ShootyCommand(Shooter shooter, Grabber grabber) {
    // Use addRequirements() here to declare subsystem dependencies.
    timestamp = 0;
    m_shooter = shooter;
    m_grabber = grabber;
    addRequirements(m_shooter, m_grabber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentState = states.START;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(currentState) {
      case START: {
        if (m_shooter.isBallReady()) {
          currentState = states.RAMPWHEEL;
        }
        break;
      }
      case RAMPWHEEL: {
        m_shooter.onWheel();
        if (m_shooter.wheelSpin() > Constants.kSetPoint - 100) {
          timestamp = Timer.getFPGATimestamp();
          currentState = states.RAISETOT;
        }
        break;
      }
      case RAISETOT: {
        m_shooter.raiseTOT(true);
        if (Timer.getFPGATimestamp() > timestamp + 2) {
          timestamp = Timer.getFPGATimestamp();
          currentState = states.LOWERTOT;
        }
        break;
      }
      case LOWERTOT: {
        m_shooter.raiseTOT(false);
        if (Timer.getFPGATimestamp() > timestamp + 3) {
          if (!m_shooter.isBallReady() && m_grabber.ballAtGrabber()) {
            currentState = states.RELEASEBALL;
          } else {
            currentState = states.END;
          }
        }
        break;
      }
      case RELEASEBALL: {
        m_grabber.grabbyGrab(false);
        currentState = states.START;
        break;
      }
      case END: {}
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.offWheel();
    m_shooter.raiseTOT(false);
    currentState = states.END;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return currentState == states.END;
  }
}
