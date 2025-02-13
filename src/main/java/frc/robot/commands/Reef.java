// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Hand;
import frc.robot.subsystems.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Reef extends Command {
  PIDController strafeController = new PIDController(1, 0.001, 0.002);
  PIDController driveController = new PIDController(1.2, 0.001, 0.002);
  PIDController rotationController = new PIDController(0.05, 0.001, 0.002);
  Hand hand;
  double strafeValue;
  double driveValue;
  double rotationValue;
  Swerve swerve;
  Pose2d targetPosition;
  Pose2d wantedError = new Pose2d(-0.1, -0.2, Rotation2d.fromDegrees(-7));
  Transform2d error;
  String LimelightName = "";
  /** Creates a new CoralStationAligment. */
  public Reef(Swerve swerve_, Hand hand_) {
    this.swerve = swerve_;
    this.hand = hand_;
    addRequirements(swerve_, hand_);
    System.out.println("Initialized");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //hand.setWantedPosition(RobotContainer.wristIntakeRotation);
    System.out.println(LimelightHelpers.getTargetCount(LimelightName));
    LimelightHelpers.SetFiducialIDFiltersOverride(LimelightName, new int[]{8}); // Only track these tag IDs
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    targetPosition = new Pose2d(
      -LimelightHelpers.getCameraPose3d_TargetSpace(LimelightName).getX(), 
      LimelightHelpers.getCameraPose3d_TargetSpace(LimelightName).getZ(),
      Rotation2d.fromDegrees(LimelightHelpers.getTX(LimelightName))
      );
      
      error = (targetPosition.minus(wantedError));
      
      strafeValue = strafeController.calculate(error.getX());
      driveValue = driveController.calculate(error.getY());
      rotationValue = rotationController.calculate(error.getRotation().getDegrees());
      swerve.drive(
        new Translation2d(driveValue, strafeValue),
        rotationValue,
        false,
        true
        );
        SmartDashboard.putNumber("X", error.getX());
        SmartDashboard.putNumber("Y", error.getY());
        SmartDashboard.putNumber("R", error.getRotation().getDegrees());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Ended");
    strafeController.reset();
    driveController.reset();
    rotationController.reset();
    LimelightHelpers.SetFiducialIDFiltersOverride(LimelightName, new int[]{1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22});
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if ((LimelightHelpers.getTargetCount(LimelightName) == 0) || (Math.abs(error.getX()) < 0.1 && Math.abs(error.getY()) < 0.1 && Math.abs(error.getRotation().getDegrees()) < 1 )){
        return true;
    }
    else
      return false;
    
  }
}
