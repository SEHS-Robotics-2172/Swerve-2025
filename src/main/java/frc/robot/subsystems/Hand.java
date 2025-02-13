package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//if (button==pressed){
//boolean win = true;
//}
public class Hand extends SubsystemBase {
    private TalonFX wristMotor;
    private SparkMax intakeMotor1;
    public SparkMax intakeMotor2;
    public PositionVoltage handPID = new PositionVoltage(0);
    double wantedPosition = 0;
    TalonFXConfiguration wristConfig = new TalonFXConfiguration();
    SparkMaxConfig intakeCCW = new SparkMaxConfig();
    SparkMaxConfig intakeCW = new SparkMaxConfig();

    public Hand(){
      wristConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
      wristConfig.Slot0.kP = 15; 
      wristConfig.Slot0.kI = 0.1;
      wristConfig.Slot0.kG = 0.1;
      wristConfig.Feedback.SensorToMechanismRatio = 10/1;
      wristConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

      intakeCCW.inverted(true);
      intakeCW.inverted(false);
      
      wristConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      intakeCCW.idleMode(IdleMode.kCoast);
      intakeCW.idleMode(IdleMode.kCoast);
      
      wristMotor = new TalonFX(Constants.Hand.wristMotorID);
      intakeMotor1 = new SparkMax(Constants.Hand.intakeMotor1ID, MotorType.kBrushless);
      intakeMotor2 = new SparkMax(Constants.Hand.intakeMotor2ID, MotorType.kBrushless);
      
      wristMotor.getConfigurator().apply(wristConfig);
    }
 @Override
  public void periodic() {
    SmartDashboard.putNumber("Wrist Position", getEncoderPosition()); 
    SmartDashboard.putNumber("Kraken stupidity", wristMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Wanted Wrist Position", wantedPosition); 
    wristMotor.setControl(handPID.withPosition(wantedPosition));
  }
  public double getEncoderPosition(){
    return intakeMotor2.getAlternateEncoder().getPosition();
  }
  public void resetToAbsolute(){
    double absolutePosition = getEncoderPosition();
    wristMotor.setPosition(absolutePosition);
}
  public void setWantedPosition(double rotations){
    wantedPosition = rotations;
  }
  public void setIntakeSpeed(double speed){
    intakeMotor1.set(speed);
    intakeMotor2.set(speed);
  }
}
