// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.commands.AutoGrabbyCommand;
import frc.robot.commands.EjectBallCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ManualShooter;
import frc.robot.commands.ShootyCommand;
import frc.robot.commands.TargetFinder;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TalonRamseteControllerAbstraction;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
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
  private XboxController m_hangerController;
  private DriveTrain m_driveTrain;
  public Grabber m_grabber;
  public Shooter m_shooter;
  public AutoGrabbyCommand m_grabCommand;
  public ShootyCommand m_shootyCommand;
  public Hanger m_hanger;
public ManualShooter m_ManualShootyCommand;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_shooter = new Shooter();
    m_xboxController = new XboxController(0);
    m_hangerController = new XboxController(1);
    m_driveTrain = new DriveTrain();
    m_grabber = new Grabber();
    m_grabCommand = new AutoGrabbyCommand(m_grabber, m_shooter);
    m_shootyCommand = new ShootyCommand(m_shooter, m_grabber);
    m_hanger = new Hanger();
    m_ManualShootyCommand=new ManualShooter(m_shooter, m_grabber);
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
  private void configureButtonBindings(){
    driversControlPlan();
  }
  private void driversControlPlan(){
    new JoystickButton(m_xboxController, Button.kA.value)
      .whileActiveOnce(m_grabCommand);
    new JoystickButton(m_xboxController, Button.kY.value)
      .whileActiveOnce(new EjectBallCommand(m_shooter, m_grabber));
    new JoystickButton(m_xboxController, Button.kB.value)
      .whileActiveOnce(
        new TargetFinder(m_driveTrain)
      );
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

      // control pannel buttons 

    new JoystickButton(m_hangerController, Button.kB.value)
      .whileActiveOnce(m_shootyCommand);
    new JoystickButton(m_hangerController, Button.kX.value)
      .whileActiveOnce(m_ManualShootyCommand);
    new JoystickButton(m_hangerController, Button.kY.value)
      .whenPressed(
        () -> m_hanger.raiseHanger() 
      )
      .whenReleased(
        () -> m_hanger.stopHang()
      );
    new JoystickButton(m_hangerController, Button.kA.value)
      .whenPressed(
        () -> m_hanger.lowerHanger() 
      )
      .whenReleased(
        () -> m_hanger.stopHang()
      );
  }

  public Command createAutoNavigationCommand(Pose2d start, List<Translation2d> waypoints, Pose2d end) {
    System.out.println("Creating Auto Command");
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(Constants.DTConsts.KS, Constants.DTConsts.KV, Constants.DTConsts.KA),
        Constants.kDriveKinematics, 10);

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(Constants.DTConsts.MAX_VELOCITY, Constants.DTConsts.MAX_ACCELERATION)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(Constants.kDriveKinematics)
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow. All units in meters.
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(start, waypoints, end, config);
    System.out.println("Generated Trajectory");
    RamseteCommand ramseteCommand = new RamseteCommand(trajectory, m_driveTrain::getPose,
        new TalonRamseteControllerAbstraction(Constants.DTConsts.RAMSETE_B, Constants.DTConsts.RAMSETE_ZETA),

        Constants.kDriveKinematics,
       
        m_driveTrain::driveVelocity, m_driveTrain);

    // Run path following command, then stop at the end.
    System.out.println("Finished Creating Auto Command");
    return ramseteCommand.andThen(() -> m_driveTrain.drive(0, 0));
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
