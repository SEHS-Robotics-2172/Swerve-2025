// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Hand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class level4Elevator extends InstantCommand {
  Elevator elevator;
  Hand hand;
  public level4Elevator(Elevator elevator, Hand hand) {
    this.elevator = elevator;
    this.hand = hand;
    addRequirements(elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.setWantedPosition(10.3);
    hand.setWantedPosition(0.15);
  }
}
