// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private TalonFX motor1;
  private TalonFX motor2;
  private SparkMax encoder;
  private PIDController elevatorController;
  double kP;
  double kI;
  double kD;
  double wantedPosition = 0;
  TalonFXConfiguration config = new TalonFXConfiguration(); 
  public Elevator() {
    kP = 0.2;
    kI = 0.2;
    kD = 0;
    elevatorController = new PIDController(kP, kI, kD);
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.Feedback.SensorToMechanismRatio = 7.75 / 2;
    motor1 = new TalonFX(Constants.Elevator.motor1ID);
    motor2 = new TalonFX(Constants.Elevator.motor2ID);
    encoder = new SparkMax(Constants.Elevator.encoderID, MotorType.kBrushed);
    motor1.getConfigurator().apply(config);
    motor2.getConfigurator().apply(config);
    motor1.setNeutralMode(NeutralModeValue.Brake);
    motor2.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void periodic() {
    double error = wantedPosition - motor1.getPosition().getValueAsDouble();
    if (error <= 0){
      elevatorController.setPID(kP/2, kI/2, kD);
    } else {
      elevatorController.setPID(kP, kI, kD);
    }
    double speed = elevatorController.calculate(error);
    motor1.setVoltage(speed);
    motor2.setVoltage(speed);
    // SmartDashboard.putNumber("Elevator Absolute Position", getRotations()); 
    // SmartDashboard.putNumber("Elevator Position", motor1.getPosition().getValueAsDouble());
    // SmartDashboard.putNumber("Elevator Wanted Position", wantedPosition); 
  }
  public void addWantedPosition(double rotations){
    wantedPosition += (rotations * 0.02);
  }
  public void setWantedPosition(double rotations){
    wantedPosition = rotations;
  }
  public double getRotations(){
    return encoder.getEncoder().getPosition();
  }
  public void resetToZero(){
    encoder.getEncoder().setPosition(0);
  }
  public void resetToAbsolute(){
    motor1.setPosition(getRotations());
    motor2.setPosition(getRotations());
  }
}
