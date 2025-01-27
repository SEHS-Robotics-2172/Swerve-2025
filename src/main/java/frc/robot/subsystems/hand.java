package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
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
    SparkMaxConfig wristConfig = new SparkMaxConfig();  
    SparkMaxConfig intakeCCW = new SparkMaxConfig();
    SparkMaxConfig intakeCW = new SparkMaxConfig();
    private AbsoluteEncoder encoder;

    public Hand(){
        wristMotor = new SparkMax(Constants.Hand.wristMotorID, MotorType.kBrushless);
        intakeMotor1 = new SparkMax(Constants.Hand.intakeMotor1ID, MotorType.kBrushless);
        intakeMotor2 = new SparkMax(Constants.Hand.intakeMotor2ID, MotorType.kBrushless);

        wristMotor.configure(wristConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        

        encoder = wristMotor.getAbsoluteEncoder();


    }
}
