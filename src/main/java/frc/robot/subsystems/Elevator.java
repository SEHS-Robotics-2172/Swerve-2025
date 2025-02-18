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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private TalonFX motor1;
  private TalonFX motor2;
  private SparkMax encoder;
  private PositionVoltage elevatorPID = new PositionVoltage(0);
  double wantedPosition = 0;
  TalonFXConfiguration config = new TalonFXConfiguration(); 
  public Elevator() {
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    config.Feedback.SensorToMechanismRatio = 7.75 / 2;
    config.Slot0.kI = 2;
    config.Slot0.kP = 6;
    config.Slot0.kD = 0;
    config.Slot0.kV = 0;
    config.Slot0.kA = 0;
    config.Slot0.kG = 0.4;
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
    PositionVoltage speed = elevatorPID.withPosition(wantedPosition);
    motor1.setControl(speed);
    motor2.setControl(speed);
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
