// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Hand;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class level4 extends Command {
  Hand hand;
  double endTime = 2;
  double timer;
  /** Creates a new level4. */
  public level4(Hand hand_) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.hand = hand_;
    addRequirements(hand_);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    timer += Robot.kDefaultPeriod;
    hand.setIntakeSpeed(-0.5);
    if (timer >= 1){
      hand.setWantedPosition(RobotContainer.wristIntakeRotation);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timer >= endTime)
      return true;
    else
      return false;
  }
}
