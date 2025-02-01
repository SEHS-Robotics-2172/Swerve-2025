package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
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
    private SparkMax intakeMotor2;
    public PIDController handPID = new PIDController(0.9, 0, 0);
    double wantedPosition = 0;
    TalonFXConfiguration wristConfig = new TalonFXConfiguration();
    SparkMaxConfig intakeCCW = new SparkMaxConfig();
    SparkMaxConfig intakeCW = new SparkMaxConfig();

    public Hand(){

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
      intakeMotor1.configure(intakeCCW, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
      intakeMotor2.configure(intakeCW, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
 @Override
  public void periodic() {
    SmartDashboard.putNumber("Wrist Position", getEncoderPosition()); 
    SmartDashboard.putNumber("Wanted Wrist Position", wantedPosition); 

    handPID.setSetpoint(wantedPosition);
    double wristSpeed = handPID.calculate(getEncoderPosition());
    wristMotor.set(wristSpeed);
  }
  public double getEncoderPosition(){
    return intakeMotor2.getAlternateEncoder().getPosition();
  }
  public void setWantedPosition(double rotations){
    wantedPosition = rotations;
  }
  public void addWantedPosition(double rotations){
    wantedPosition += rotations * handPID.getPeriod();
  }
  public void setIntakeSpeed(double speed){
    intakeMotor1.set(speed);
    intakeMotor2.set(speed);
  }
}
