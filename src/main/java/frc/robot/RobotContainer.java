// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.commands.AutoGrabbyCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private XboxController m_xboxController;
  private DriveTrain m_driveTrain;
  public Grabber m_grabber;
  public Shooter m_shooter;
  public AutoGrabbyCommand m_grabCommand;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_shooter = new Shooter();
    m_xboxController = new XboxController(0);
    m_driveTrain = new DriveTrain();
    m_grabber = new Grabber();
    m_grabCommand = new AutoGrabbyCommand(m_grabber);
    // Configure the button bindings
    configureButtonBindings();

    m_driveTrain.setDefaultCommand(
      new RunCommand(
        () -> 
          m_driveTrain.drive(m_xboxController.getLeftX(),-m_xboxController.getLeftY()), m_driveTrain
        )
        );
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_xboxController, Button.kRightBumper.value)
        .whenPressed(
          () -> m_grabber.grabbyGrab(true)
        )
        .whenReleased(
          () -> m_grabber.grabbyGrab(false)
        );
    new JoystickButton(m_xboxController, Button.kLeftBumper.value)
      .whenPressed(
        () -> m_grabber.lowerGrabber(true)
      )
      .whenReleased(
        () -> m_grabber.lowerGrabber(false)
      );
    new JoystickButton(m_xboxController, Button.kA.value)
      .whileActiveOnce(m_grabCommand);

      new JoystickButton(m_xboxController, Button.kX.value)
      .whenPressed(
        () -> m_shooter.shootyShoot(true)
      )
      .whenReleased(
        () -> m_shooter.shootyShoot(false)
      );
      new JoystickButton(m_xboxController, Button.kY.value)
      .whenPressed(
        () -> m_shooter.onWheel()
      )
      .whenReleased(
        () -> m_shooter.offWheel()
      );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
