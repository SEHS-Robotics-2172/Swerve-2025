// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private TalonFX motor1;
  private TalonFX motor2;
  TalonFXConfiguration config = new TalonFXConfiguration(); 
  public Elevator() {
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motor1 = new TalonFX(Constants.Elevator.motor1ID);
    motor2 = new TalonFX(Constants.Elevator.motor2ID);
    motor1.getConfigurator().apply(config);
    motor2.getConfigurator().apply(config);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
