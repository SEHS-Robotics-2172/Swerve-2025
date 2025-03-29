// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private TalonFX climbMotor;
  private PIDController pidClimb = new PIDController(15*2, 0, 0);
  public double wantedPosition = 0;
  private CANcoder encoder = new CANcoder(Constants.Climber.climbEncoderID);
  private CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

  TalonFXConfiguration climb = new TalonFXConfiguration();
  public Climber(){
    climb.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    climb.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    climbMotor = new TalonFX(Constants.Climber.climbMotorID);
    climbMotor.getConfigurator().apply(climb);
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    encoder.getConfigurator().apply(encoderConfig);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double error = wantedPosition - getEncoderPosition();
    double speed = -pidClimb.calculate(error);
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
    return -encoder.getPosition().getValueAsDouble();
  }
  
}
 