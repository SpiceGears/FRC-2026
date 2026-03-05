package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {

    private final SparkMax climbMotor;
    private final SparkMax followerMotor;

    public ClimbSubsystem() {
        climbMotor = new SparkMax(41, MotorType.kBrushless);
        followerMotor = new SparkMax(42, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config
            .smartCurrentLimit(60) 
            .idleMode(IdleMode.kBrake) 
            .inverted(false); 

        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig
            .apply(config) 
            .follow(climbMotor, true); 

        climbMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        followerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command runManualCommand(double volts) {
        return Commands.startEnd(
            () -> climbMotor.setVoltage(volts), 
            () -> climbMotor.setVoltage(0),     
            this
        ).withName("Climb.Manual");
    }

    public Command open() {
        return Commands.startEnd(
            () -> climbMotor.setVoltage(-10), 
            () -> climbMotor.setVoltage(0)
            );
    }
}