// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private TalonFX motor1;
  private TalonFX motor2;
  private PIDController elevatorPID = new PIDController(0.03, 0.0012, 0);
  double wantedPosition = 0;
  TalonFXConfiguration config = new TalonFXConfiguration(); 
  public Elevator() {
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motor1 = new TalonFX(Constants.Elevator.motor1ID);
    motor2 = new TalonFX(Constants.Elevator.motor2ID);
    motor1.getConfigurator().apply(config);
    motor2.getConfigurator().apply(config);
    motor1.setPosition(0);
    motor2.setPosition(0);
    motor1.setNeutralMode(NeutralModeValue.Brake);
    motor2.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void periodic() {
    elevatorPID.setSetpoint(wantedPosition);
    double speed = elevatorPID.calculate(motor1.getPosition().getValueAsDouble());
    motor1.set(speed);
    motor2.set(speed);
    SmartDashboard.putNumber("Wanted Position", wantedPosition);
    SmartDashboard.putNumber("Current Position", motor1.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Elevator Speed", speed); 
  }
  public void addWantedPosition(double rotations){
    wantedPosition += (7.75 * rotations * elevatorPID.getPeriod());
  }
  public void setWantedPosition(double rotations){
    wantedPosition = rotations;
  }
}
