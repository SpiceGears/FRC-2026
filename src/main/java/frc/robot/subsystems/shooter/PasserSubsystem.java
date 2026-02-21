// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortMap;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class PasserSubsystem extends SubsystemBase {

  public PasserSubsystem instance;

  // SmartMotorControllerConfig passerMotorConfig = new SmartMotorControllerConfig(this)
  // .withControlMode(ControlMode.CLOSED_LOOP)
  // .withIdleMode(MotorMode.COAST)
  // .withMotorInverted(true)
  // .withTelemetry("ShooterPasserMotor", TelemetryVerbosity.MID)
  // .withClosedLoopController(40,0,0, RPM.of(6000), DegreesPerSecondPerSecond.of(100))
  // .withSimClosedLoopController(40,0,0, RPM.of(6000), DegreesPerSecondPerSecond.of(100))
  // .withFeedforward(new SimpleMotorFeedforward(0, 0))
  // .withSimFeedforward(new SimpleMotorFeedforward(0, 0))
  // .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
  // .withClosedLoopRampRate(Seconds.of(0.25))
  // .withOpenLoopRampRate(Seconds.of(0.25))
  // .withStatorCurrentLimit(Amps.of(30));

  private SparkMax passerMotor = new SparkMax(PortMap.SHOOTER_PASSER_MOTOR_ID, MotorType.kBrushless);
  private SparkMaxConfig passerMotorConfig = new SparkMaxConfig();

  //SmartMotorController passerController = new SparkWrapper(passerMotor, DCMotor.getNEO(1), passerMotorConfig);
  /** Creates a new ShooterPasserSubsystem. */
  public PasserSubsystem() 
  {
    if (instance != null) {
      //throw new IllegalStateException("ShooterPasserSubsystem instance already exists!");
      return;
    }

    passerMotorConfig
    .idleMode(IdleMode.kCoast)
    .smartCurrentLimit(20)
    .closedLoopRampRate(0.25)
    .openLoopRampRate(0.25)
    .inverted(true);

    passerMotor.configure(passerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    instance = this;
  }

  @Override
  public void periodic() {
    //passerController.updateTelemetry();
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    //passerController.simIterate();
    // This method will be called once per scheduler run during simulation
  }


  public void setPassingSpeed(double speed) 
  {
    passerMotor.set(MathUtil.clamp(speed, -1, 1));
    //passerController.setVelocity(speed);
  }

  public Command passShooter(DoubleSupplier speed) 
  {
    return runEnd(
      () -> 
      {
        setPassingSpeed(speed.getAsDouble());
      }, 
      () -> {setPassingSpeed(0);});
  }

  public void stopPasser() 
  {
    //passerController.setDutyCycle(0);
    passerMotor.stopMotor();
  }

  public void passFuelToShooter(BooleanSupplier enabled) 
  {
      if (enabled.getAsBoolean()) 
      {
        this.setPassingSpeed(1.00);
      } else 
      {
        this.stopPasser();
      }
  }

  public Command passFuelToShooterCommand(BooleanSupplier enabled) 
  {
    return run(() -> 
    {
      if (enabled.getAsBoolean()) 
      {
        this.setPassingSpeed(0.90);
      } else 
      {
        this.stopPasser();
      }
    });
  }
}

