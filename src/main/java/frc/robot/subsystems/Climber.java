// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private SparkMax climbMotor;
  private PIDController pidClimb = new PIDController(1, 0.1, 0);
  public double wantedPosition = 0;

  SparkMaxConfig climb = new SparkMaxConfig();
  public Climber(){
    climb.idleMode(IdleMode.kCoast);
    climbMotor = new SparkMax(Constants.Climber.climbMotorID, MotorType.kBrushless);
    climbMotor.configure(climb, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    climb.inverted(false);
    climbMotor.configure(climb, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double error = wantedPosition - getEncoderPosition();
    double speed = pidClimb.calculate(error);
    climbMotor.setVoltage(speed);

    SmartDashboard.putNumber("Climber Absolute Position", getEncoderPosition()); 
    SmartDashboard.putNumber("Climber Wanted Position", wantedPosition);
    SmartDashboard.putNumber("Climber Speedy", speed);
    SmartDashboard.putNumber("Climber Error", error);

  }

  public void setPosition(double position) {
    wantedPosition = position;
  }

  public void addWantedPosition(double speed){
    wantedPosition += Robot.kDefaultPeriod * 0.25 * speed;
  }

  public double getEncoderPosition() {
    // encoder nonsense
    return climbMotor.getAlternateEncoder().getPosition();
  }
  
}
 