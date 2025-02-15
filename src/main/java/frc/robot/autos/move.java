// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class move extends Command {
  /** Creates a new move. */
  Swerve swerve;
  double timer;
  public move(Swerve swerve_) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve_;
    addRequirements(swerve_);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = 1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    timer -= Robot.kDefaultPeriod;
    swerve.drive(new Translation2d(1, 0), 0, true, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timer <= 0)
      return true;
    else
      return false;
  }
}
