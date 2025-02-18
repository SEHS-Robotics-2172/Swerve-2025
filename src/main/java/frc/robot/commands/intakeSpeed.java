// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hand;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class intakeSpeed extends Command {
  /** Creates a new intakeSpeed. */
  Hand hand;
  DoubleSupplier speed;
  public intakeSpeed(Hand hand_, DoubleSupplier speed_) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.speed = speed_;
    this.hand = hand_;
    addRequirements(hand_);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speedAsDouble = speed.getAsDouble();
    hand.setIntakeSpeed(speedAsDouble);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {hand.setIntakeSpeed(0);}
}
