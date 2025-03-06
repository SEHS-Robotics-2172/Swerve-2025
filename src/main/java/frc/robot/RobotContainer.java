package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final SendableChooser<Command> autochooser;
    /* Controllers */
    public final XboxController driver = new XboxController(0);
    public final XboxController co_driver = new XboxController(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    public static final double wristScoreTRotation = 0.36;
    public static final double wristIntakeRotation = 0.61;

    /* Driver Buttons */
    private final Trigger coralStation = new JoystickButton(driver, XboxController.Button.kX.value);
    private final Trigger zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final Trigger robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final Trigger reefLeftButton = new Trigger(() -> driver.getPOV() == 270);
    private final Trigger reefRightButton = new Trigger(() -> driver.getPOV() == 90);
    private final Trigger SlowDownButton = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    
    /* Co-Driver Buttons */
    private final Trigger set0 = new JoystickButton(co_driver, XboxController.Button.kRightBumper.value);
    private final Trigger setLevelTwo = new Trigger(co_driver::getAButton);
    private final Trigger setLevelThree = new Trigger(co_driver::getBButton);
    private final Trigger setLevelFour = new Trigger(co_driver::getYButton);
    private final Trigger intakePosition = new Trigger(co_driver::getXButton);
    private final Trigger level4Score = new Trigger(co_driver::getLeftBumperButton);
    private final Trigger climberup = new Trigger(driver::getAButton);
    private final Trigger climberdown = new Trigger(driver::getBButton);

    /* Subsystems */
    public Hand hand = new Hand();
    public final Swerve s_Swerve = new Swerve();
    public final Elevator elevator = new Elevator();
    public final Climber climber = new Climber();

    public final InstantCommand intakePositionCommand = new InstantCommand(() -> hand.setWantedPosition(wristIntakeRotation));

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)),
        new Pose2d(3.0, 1.0, Rotation2d.fromDegrees(0))
    );
    PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0);
    PathPlannerPath path = new PathPlannerPath(
        waypoints,
        constraints,
     null,
      new GoalEndState(0.0, Rotation2d.kZero));

      public Command commandFromPath(PathPlannerPath path){
        return AutoBuilder.followPath(path);
      }

    

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        path.preventFlipping = true;
        NamedCommands.registerCommand("ReefLeft", new ReefLeft(s_Swerve, hand));
        NamedCommands.registerCommand("ReefRight", new ReefRight(s_Swerve, hand));
        NamedCommands.registerCommand("Shoot", new shoot(hand));
        NamedCommands.registerCommand("IntakePos", intakePositionCommand);
        autochooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autochooser);
        

        hand.setDefaultCommand(new intakeSpeed(
            hand, () -> (co_driver.getLeftTriggerAxis()-co_driver.getRightTriggerAxis()),
            () -> co_driver.getPOV() == 0,
            () -> co_driver.getPOV() == 180

            )
        );
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );
        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

        SlowDownButton.whileTrue(new InstantCommand(() -> {
            // hand.resetToAbsolute();
            elevator.resetToAbsolute();
        }));

        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        coralStation.onTrue(new CoralStationAligment(s_Swerve, hand));
        reefLeftButton.onTrue(new ReefLeft(s_Swerve, hand));
        reefRightButton.onTrue(new ReefRight(s_Swerve, hand));

        set0.onTrue(new InstantCommand(() -> elevator.setWantedPosition(0)));

        setLevelTwo.onTrue(new InstantCommand(() -> hand.setWantedPosition(wristScoreTRotation)));

        setLevelThree.onTrue(new InstantCommand(() -> {
            elevator.setWantedPosition(3.8);
            hand.setWantedPosition(wristScoreTRotation);
        }));

        setLevelFour.onTrue(new InstantCommand(() -> {
            elevator.setWantedPosition(10.3);
            hand.setWantedPosition(0.3);
        }));

        intakePosition.onTrue(intakePositionCommand);

        level4Score.onTrue(new level4(hand));
      
        climberup.whileTrue(new InstantCommand(() -> {
            climber.setPosition(6.52);
            System.out.println("up");
        }));
        
        climberdown.whileTrue(new InstantCommand(() -> {
            climber.setPosition(0); 
            System.out.println("down");
        }));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return autochooser.getSelected();
    }
}
