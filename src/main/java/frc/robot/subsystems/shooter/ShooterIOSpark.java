package frc.robot.subsystems.shooter;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

// import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

public class ShooterIOSpark implements ShooterIO {

  SparkMax shooterMotor = new SparkMax(0, MotorType.kBrushless);

  SparkClosedLoopController shooterController;
  RelativeEncoder shooterEncoder;

  public ShooterIOSpark() {
    SparkMaxConfig config = new SparkMaxConfig();

    config.idleMode(IdleMode.kCoast).smartCurrentLimit(40, 40).inverted(false);
  }
}
