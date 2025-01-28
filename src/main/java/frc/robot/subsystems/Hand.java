package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//if (button==pressed){
//boolean win = true;
//}
public class Hand extends SubsystemBase {
    private SparkMax wristMotor;
    private SparkMax intakeMotor1;
    private SparkMax intakeMotor2;
    private PIDController handPID = new PIDController(0, 0, 0);
    double wantedPosition = 0;
    SparkMaxConfig wristConfig = new SparkMaxConfig();  
    SparkMaxConfig intakeCCW = new SparkMaxConfig();
    SparkMaxConfig intakeCW = new SparkMaxConfig();

    public Hand(){
        wristConfig.inverted(false);
        intakeCCW.inverted(true);
        intakeCW.inverted(false);

        wristConfig.idleMode(IdleMode.kBrake);
        intakeCCW.idleMode(IdleMode.kCoast);
        intakeCW.idleMode(IdleMode.kCoast);

        wristMotor = new SparkMax(Constants.Hand.wristMotorID, MotorType.kBrushless);
        intakeMotor1 = new SparkMax(Constants.Hand.intakeMotor1ID, MotorType.kBrushless);
        intakeMotor2 = new SparkMax(Constants.Hand.intakeMotor2ID, MotorType.kBrushless);

        wristMotor.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakeMotor1.configure(intakeCCW, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakeMotor2.configure(intakeCW, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
 @Override
  public void periodic() {
    handPID.setSetpoint(wantedPosition);
    double wristSpeed = handPID.calculate(getEncoderPosition());
    wristMotor.set(wristSpeed);
  }
  public double getEncoderPosition(){
    return wristMotor.getAlternateEncoder().getPosition();
  }
  public void addWantedPosition(double rotations){
    wantedPosition += (7.75 * rotations * handPID.getPeriod());
  }
  public void setWantedPosition(double rotations){
    wantedPosition = rotations;
  }
}
