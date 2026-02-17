// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  SmartMotorControllerConfig shooterMotorConfig = new SmartMotorControllerConfig(this)
  .withControlMode(ControlMode.CLOSED_LOOP)
  .withClosedLoopController(
    50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
  .withFeedforward(new SimpleMotorFeedforward(0, 0)
  ).withSimFeedforward(new SimpleMotorFeedforward(0, 0))
  .withTelemetry("ShooterMotor", TelemetryVerbosity.MID)
  .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
  .withMotorInverted(false)
  .withIdleMode(MotorMode.COAST)
  .withStatorCurrentLimit(Amps.of(40));

  private final SparkMax shooterSpark = new SparkMax(Constants.PortMap.SHOOTER_MOTOR_ID, MotorType.kBrushless);

  private SmartMotorController shooterController = new SparkWrapper(shooterSpark, DCMotor.getNEO(1), shooterMotorConfig);

  private final FlyWheelConfig shooterMechanismConfig = new FlyWheelConfig(shooterController)
  .withDiameter(Centimeter.of(10))
  .withMass(Kilograms.of(1.5))
  .withUpperSoftLimit(RPM.of(6000))
  .withTelemetry("ShooterMechanism", TelemetryVerbosity.HIGH);
  

  private FlyWheel shooter = new FlyWheel(shooterMechanismConfig);

  private boolean shooterEnabled = false;


  public ShooterSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shooter.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    shooter.simIterate();
  }


  private AngularVelocity getVelocity() {
    return shooter.getSpeed();
  }

  private Command setVelocity(AngularVelocity velocity) 
  {
    return shooter.setSpeed(velocity);
  }

  public Command toogleShooter() 
  {
    if (!shooterEnabled)
    {
      shooterEnabled = true;
      return setVelocity(RPM.of(5000));
    }
    else
    {
      shooterEnabled = false;
      return setVelocity(RPM.of(0));
    }
  }

  private void setVelocitySetpoint(AngularVelocity setpoint) 
  {
    shooter.setMechanismVelocitySetpoint(setpoint);
  }

}