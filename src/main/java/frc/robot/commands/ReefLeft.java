// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.subsystems.Hand;
import frc.robot.subsystems.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ReefLeft extends Command {
  PIDController strafeController = new PIDController(1, 0.001, 0.002);
  PIDController driveController = new PIDController(1.2, 0.001, 0.002);
  PIDController rotationController = new PIDController(0.075, 0.001, 0.002);
  Hand hand;
  double strafeValue;
  double driveValue;
  double rotationValue;
  Swerve swerve;
  Pose2d targetPosition;
  Pose2d wantedError = new Pose2d(0, -0.3, Rotation2d.fromDegrees(0));
  Transform2d error;
  String LimelightName = "";
  double timer;
  boolean pid;
  /** Creates a new Reef. */
  public ReefLeft(Swerve swerve_, Hand hand_) {
    this.swerve = swerve_;
    this.hand = hand_;
    addRequirements(swerve_, hand_);
    System.out.println("Initialized");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid = true;
    timer = 1;
    //hand.setWantedPosition(RobotContainer.wristIntakeRotation);
    System.out.println(LimelightHelpers.getTargetCount(LimelightName));
    LimelightHelpers.SetFiducialIDFiltersOverride(LimelightName, new int[]{6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22}); // Only track these tag IDs
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


  if (pid){
    targetPosition = new Pose2d(
      -LimelightHelpers.getCameraPose3d_TargetSpace(LimelightName).getX(), 
      LimelightHelpers.getCameraPose3d_TargetSpace(LimelightName).getZ(),
      Rotation2d.fromDegrees(LimelightHelpers.getTX(LimelightName))
      );
    error = (targetPosition.minus(wantedError));
    strafeValue = strafeController.calculate(error.getX());
    driveValue = driveController.calculate(error.getY());
    rotationValue = -rotationController.calculate(error.getRotation().getDegrees());
    swerve.drive(
      new Translation2d(driveValue, strafeValue),
      rotationValue,
      false,
      true
      );
  }


  if (!pid){
    timer -= Robot.kDefaultPeriod;
    if(timer >= 0.25)
      swerve.drive(new Translation2d(0, 0.26), 0, false, true);
    else
      swerve.drive(new Translation2d(1, 0), 0, false, true);
  }
  
  // SmartDashboard.putNumber("X", error.getX());
  // SmartDashboard.putNumber("Y", error.getY());
  // SmartDashboard.putNumber("R", error.getRotation().getDegrees());


  if(((LimelightHelpers.getTargetCount(LimelightName) == 0) || (Math.abs(error.getX()) < 0.1 && Math.abs(error.getY()) < 0.1 && Math.abs(error.getRotation().getDegrees()) < 0.5 )))
    pid = false;
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

    if (timer <=0){
        return true;
    }
    else
      return false;
    
  }
}
