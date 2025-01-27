package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//if (button==pressed){
//boolean win = true;
//}
public class hand extends SubsystemBase {
    private SparkMax motor1;
    private PIDController handPID = new PIDController(0, 0, 0);

    SparkMaxConfig config = new SparkMaxConfig();       

    public void wrist(){
        motor1 = new SparkMax(1, MotorType.kBrushless);
        motor1.configure(config, null, null);
        //if grady here= false;
        // sustem.print(grady is a dingo);
        
    }
}
