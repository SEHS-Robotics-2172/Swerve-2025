// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralStationAligment extends Command {
  PIDController strafeController = new PIDController(0.1, 0.001, 0.002);
  PIDController driveController = new PIDController(0.1, 0.001, 0.002);
  Swerve swerve;
  Pose3d targetPosition;
  Pose3d wantedError = new Pose3d(0, 1, 0, new Rotation3d(0, 0, 0));
  Pose3d error;
  /** Creates a new CoralStationAligment. */
  public CoralStationAligment(Swerve swerve_) {
    this.swerve = swerve_;
    addRequirements(swerve_);
    targetPosition = LimelightHelpers.getTargetPose3d_CameraSpace("");
    strafeController.setTolerance(0.05);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    error = new Pose3d(targetPosition.minus(wantedError).getX(),
    targetPosition.minus(wantedError).getY(),
    targetPosition.minus(wantedError).getZ(),
    targetPosition.getRotation().minus(wantedError.getRotation()));
    swerve.drive(
      new Translation2d(strafeController.calculate(error.getX()), driveController.calculate(error.getZ())),
      0,
      true,
      true
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    strafeController.reset();
    driveController.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((LimelightHelpers.getTargetCount("") == 0) || (strafeController.atSetpoint() && driveController.atSetpoint()))
      return true;
    else
      return false;
  }
}
