// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Detector;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.subsystems.Hand;
import frc.robot.subsystems.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ReefRightLevel4 extends Command {
  PIDController strafeController = new PIDController(3, 0, 0);
  PIDController driveController = new PIDController(3, 0, 0);
  ProfiledPIDController rotationController = new ProfiledPIDController(4, 0, 0, (new Constraints(6.26, 3.14)));
  HolonomicDriveController controller = new HolonomicDriveController(strafeController, driveController, rotationController);
  Hand hand;
  double strafeValue;
  double driveValue;
  double rotationValue;
  Swerve swerve;
  Pose2d robotPosition;
  Pose2d wantedError = new Pose2d(0.12, -0.5, Rotation2d.fromDegrees(0));
  State goalState = new State(0, 0, 0, wantedError, 0);
  Transform2d error;
  String LimelightName = "";
  double timer;
  boolean pid;
  double endTimer;
  /** Creates a new Reef. */
  public ReefRightLevel4(Swerve swerve_, Hand hand_) {
    this.swerve = swerve_;
    this.hand = hand_;
    addRequirements(swerve_, hand_);
    System.out.println("Initialized");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //LimelightHelpers.SetFidcuial3DOffset(LimelightName, 0.2, 0, 0);
    pid = true;
    timer = 0.6;
    endTimer = 10;
    //hand.setWantedPosition(RobotContainer.wristIntakeRotation);
    controller.setTolerance(new Pose2d(0.05, 0.05, Rotation2d.fromRotations(0.03)));

    System.out.println(LimelightHelpers.getTargetCount(LimelightName));
    LimelightHelpers.SetFiducialIDFiltersOverride(LimelightName, new int[]{6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22}); // Only track these tag IDs
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    endTimer -= Robot.kDefaultPeriod;

  if (pid){
    robotPosition = new Pose2d(
      LimelightHelpers.getCameraPose3d_TargetSpace(LimelightName).getX(), 
      LimelightHelpers.getCameraPose3d_TargetSpace(LimelightName).getZ(),
      Rotation2d.fromRadians(-LimelightHelpers.getCameraPose3d_TargetSpace(LimelightName).getRotation().getY())
      );
    ChassisSpeeds speeds = controller.calculate(robotPosition, goalState, Rotation2d.kZero);
    swerve.setModuleStates(Constants.Swerve.swerveKinematics.toSwerveModuleStates(speeds));
  }


  if (!pid){
    timer -= Robot.kDefaultPeriod;
    swerve.drive(new Translation2d(0.5, 0), 0, false, true);
  }
  
  // SmartDashboard.putNumber("X", error.getX());
  // SmartDashboard.putNumber("Y", error.getY());
  // SmartDashboard.putNumber("R", error.getRotation().getDegrees());


  if((LimelightHelpers.getTargetCount(LimelightName) == 0) || controller.atReference() || endTimer <= 0)
    pid = false;
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Ended");
    //LimelightHelpers.SetFidcuial3DOffset(LimelightName, 0, 0, 0);
    LimelightHelpers.SetFiducialIDFiltersOverride(LimelightName, new int[]{1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22});
    swerve.setDriveVoltage(0);
    // swerve.getCurrentCommand().cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (timer <= 0){
        return true;
    }
    else
      return false;
    
  }
}
