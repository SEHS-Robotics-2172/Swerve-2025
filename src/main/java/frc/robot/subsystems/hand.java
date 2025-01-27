package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//if (button==pressed){
//boolean win = true;
//}
public class Hand extends SubsystemBase {
    private SparkMax motor1;
    private PIDController handPID = new PIDController(0, 0, 0);
    SparkMaxConfig config = new SparkMaxConfig();  

    public Hand(){
        motor1 = new SparkMax(1, MotorType.kBrushless);
        motor1.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        
    }
}
