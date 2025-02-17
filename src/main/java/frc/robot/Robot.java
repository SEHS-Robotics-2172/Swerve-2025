// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.move;
import frc.robot.autos.moveKT;
import frc.robot.commands.ReefLeft;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Hand;
import frc.robot.subsystems.Swerve;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static final CTREConfigs ctreConfigs = new CTREConfigs();
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private Hand hand;

  private Elevator elevator;
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private static final String kCustomAuto2 = "My Auto 2";
  private static final String kCustomAuto3 = "My Auto 3";
  private static final String kCustomAuto4 = "My Auto 4";
  private static final String kCustomAuto5 = "My Auto 5";
  private static final String kCustomAuto6 = "My Auto 6";
  private static final String kCustomAuto7 = "My Auto 7";
  private static final String kCustomAuto0 = "Do Nothing";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private XboxController co_driver;
  
  private Swerve swerve;
  TalonFX test;
  public int autoNumber = 0;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    hand = m_robotContainer.hand;
    swerve = m_robotContainer.s_Swerve;
    elevator = m_robotContainer.elevator;
    co_driver =  m_robotContainer.co_driver;

    m_chooser.setDefaultOption("Coral Left", kDefaultAuto);
    m_chooser.addOption("Shoot", kCustomAuto);
    m_chooser.addOption("Do Nothing", kCustomAuto0);
    m_chooser.addOption("Middle 2 and leave", kCustomAuto2);
    m_chooser.addOption("Blue Middle 3 and leave", kCustomAuto3);
    m_chooser.addOption("Blue Left 3 and leave", kCustomAuto4);
    m_chooser.addOption("Right 3 and leave", kCustomAuto5);
    SmartDashboard.putData("Auto choices", m_chooser);
    //if (robot.existing == true) {
    //  robot.fix();
    //  robot.work();
    //}

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);
    switch (m_autoSelected) {
      case kCustomAuto:
        autoNumber = 1;
        break;
      case kCustomAuto2:
       autoNumber = 2;
       break;
      case kCustomAuto3:
       autoNumber = 3;
       break;
      case kCustomAuto4:
       autoNumber = 4;
       break;
      case kCustomAuto5:
       autoNumber = 5;
       break;
      case kCustomAuto6:
       autoNumber = 6;
       break;
      case kCustomAuto7:
       autoNumber = 7;
       break;
      case kCustomAuto0:
       autoNumber = -1;
       break;
      case kDefaultAuto:
        m_autonomousCommand = new SequentialCommandGroup(
          new Command[]{
            new moveKT(swerve),
            new InstantCommand(() -> hand.setWantedPosition(RobotContainer.wristScoreTRotation)),
            new ReefLeft(swerve, hand),
            new InstantCommand(() -> hand.setIntakeSpeed(-0.5))
          }
        );
      break;
      default:
        autoNumber = 0;
        break;
    }

    //m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    //hand.resetToAbsolute();
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    elevator.resetToAbsolute();
    hand.resetToAbsolute();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //System.out.println(LimelightHelpers.getTargetCount("limelight-old"));
    LimelightHelpers.SetRobotOrientation("", swerve.gyro.getYaw().getValueAsDouble(), 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation("limelight-old", swerve.gyro.getYaw().getValueAsDouble(), 0, 0, 0, 0, 0);
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    
  }
  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
