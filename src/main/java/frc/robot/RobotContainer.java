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

    /* Driver Buttons */
    private final Trigger intakePosition = new Trigger(() -> (co_driver.getAButton()));

    private final Trigger coralStation = new JoystickButton(driver, XboxController.Button.kX.value);
    private final Trigger zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final Trigger robotCentric = new JoystickButton(co_driver, XboxController.Button.kLeftBumper.value);
    private final Trigger set0 = new Trigger(() -> (co_driver.getPOV() == 0));
    private final Trigger set10 = new Trigger(() -> (co_driver.getPOV() ==270));
    private final Trigger set20 = new Trigger(() -> (co_driver.getPOV() == 180));
    private final Trigger set30 = new Trigger(() -> (co_driver.getPOV() == 90));
    private final Trigger set40 = new Trigger(() -> (co_driver.getRightStickButton()));

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
        set10.onTrue(new InstantCommand(() -> elevator.setWantedPosition(15)));
        set20.onTrue(new InstantCommand(() -> elevator.setWantedPosition(30)));
        set30.onTrue(new InstantCommand(() -> elevator.setWantedPosition(45)));
        set40.onTrue(new InstantCommand(() -> elevator.setWantedPosition(63)));
        intakePosition.whileTrue(new InstantCommand(() -> hand.setWantedPosition(0.42)));
        intakePosition.whileFalse(new InstantCommand(() -> hand.setWantedPosition(0.20)));
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
