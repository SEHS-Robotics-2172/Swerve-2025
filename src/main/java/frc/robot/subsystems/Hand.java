package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
//if (button==pressed){
//boolean win = true;
//}
public class Hand extends SubsystemBase {
    private TalonFX wristMotor;
    public SparkMax intakeMotor1;
    public SparkMax intakeMotor2;
    public PIDController handController;
    double wantedPosition = 0;
    TalonFXConfiguration wristConfig = new TalonFXConfiguration();
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    SparkMaxConfig intakeCCW = new SparkMaxConfig();
    SparkMaxConfig intakeCW = new SparkMaxConfig(); 
    CANcoder encoder;

    public Hand(){
      handController = new PIDController(10, 7, 0);
      wristConfig.Feedback.SensorToMechanismRatio = 10;
      wristConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.8;

      intakeCCW.inverted(true);
      intakeCW.inverted(false);
      
      wristConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      intakeCCW.idleMode(IdleMode.kCoast);
      intakeCW.idleMode(IdleMode.kCoast);
      
      wristMotor = new TalonFX(Constants.Hand.wristMotorID);
      intakeMotor1 = new SparkMax(Constants.Hand.intakeMotor1ID, MotorType.kBrushless);
      intakeMotor2 = new SparkMax(Constants.Hand.intakeMotor2ID, MotorType.kBrushless);
      encoder = new CANcoder(Constants.Hand.wristEncoderID);

      intakeMotor1.configure(intakeCCW, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
      intakeMotor2.configure(intakeCW, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
      
      wristMotor.getConfigurator().apply(wristConfig);
      wristMotor.setPosition(0);
      encoderConfig.MagnetSensor.MagnetOffset = -0.201904296875;
      encoder.getConfigurator().apply(encoderConfig);
      resetToAbsolute();
    }
 @Override
  public void periodic() {
    double error = getEncoderPosition() - wantedPosition;
    SmartDashboard.putNumber("Wrist Position", getEncoderPosition()); 
    SmartDashboard.putNumber("Kraken stupidity", wristMotor.getPosition().getValueAsDouble());
    wristMotor.setVoltage(handController.calculate(error));
  }
  public double getEncoderPosition(){
    return encoder.getAbsolutePosition().getValueAsDouble();
  }
  public void resetToAbsolute(){
    double absolutePosition = getEncoderPosition();
    wristMotor.setPosition(absolutePosition);
}
  public void setWantedPosition(double rotations){
    wantedPosition = rotations;
  }
  public void addWantedPosition(double speed){
    wantedPosition += Robot.kDefaultPeriod * 0.25 * speed;
  }
  public void setIntakeSpeed(double speed){
    intakeMotor1.set(speed);
    intakeMotor2.set(speed);
  }
}
