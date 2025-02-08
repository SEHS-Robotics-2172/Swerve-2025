package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    public final XboxController driver = new XboxController(1);
    public final XboxController co_driver = new XboxController(2);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    private final double wristScoreTRotation = 0.2;
    private final double wristIntakeRotation = 0.4;

    /* Driver Buttons */
    private final Trigger coralStation = new JoystickButton(driver, XboxController.Button.kX.value);
    private final Trigger zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final Trigger robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final Trigger set0 = new JoystickButton(co_driver, XboxController.Button.kRightBumper.value);
    private final Trigger setLevelTwo = new Trigger(() -> (co_driver.getAButton()));
    private final Trigger setLevelThree = new Trigger(() -> (co_driver.getBButton()));
    private final Trigger setLevelFour = new Trigger(() -> (co_driver.getYButton()));
    private final Trigger scorePosition = new Trigger(() -> (co_driver.getXButton()));
    
   

    /* Subsystems */
    public Hand hand = new Hand();
    public final Swerve s_Swerve = new Swerve();
    public final Elevator elevator = new Elevator();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -co_driver.getRawAxis(translationAxis), 
                () -> -co_driver.getRawAxis(strafeAxis), 
                () -> -co_driver.getRawAxis(rotationAxis), 
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
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        coralStation.onTrue(new CoralStationAligment(s_Swerve));

        set0.onTrue(new InstantCommand(() -> elevator.setWantedPosition(0)));

        setLevelTwo.onTrue(new InstantCommand(() -> hand.setWantedPosition(wristScoreTRotation)));

        setLevelThree.onTrue(new InstantCommand(() -> {
            elevator.setWantedPosition(2);
            hand.setWantedPosition(wristScoreTRotation);
        }));

        setLevelFour.onTrue(new InstantCommand(() -> {
            elevator.setWantedPosition(4);
            hand.setWantedPosition(0);
        }));

        scorePosition.onTrue(new InstantCommand(() -> hand.setWantedPosition(wristIntakeRotation)));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(s_Swerve);
    }
}
