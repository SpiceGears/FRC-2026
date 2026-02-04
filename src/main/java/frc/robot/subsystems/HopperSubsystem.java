// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class HopperSubsystem extends SubsystemBase {

  @AutoLog
  public static class HopperInputs
  {
    public AngularVelocity velocity = DegreesPerSecond.of(0);
    public Voltage         volts    = Volts.of(0);
    public Current         current  = Amps.of(0);
  }

  private final HopperInputs hopperInputs = new HopperInputs();

  private final SparkMax controller = new SparkMax(30, MotorType.kBrushless);

  private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      .withIdleMode(MotorMode.COAST)
      .withTelemetry("HopperMotor", TelemetryVerbosity.HIGH)
      .withStatorCurrentLimit(Amps.of(40))
      .withMotorInverted(false) //may need inversion
      .withControlMode(ControlMode.OPEN_LOOP);

  private final SmartMotorController motor = new SparkWrapper(controller, DCMotor.getNeo550(1), motorConfig);

  private void updateInputs()
  {
    hopperInputs.velocity = motor.getMechanismVelocity();
    hopperInputs.volts = motor.getVoltage();
    hopperInputs.current = motor.getStatorCurrent();
  }

  public HopperSubsystem() {}

  public AngularVelocity getVelocity()
  {
    return hopperInputs.velocity;
  }

  public Command setVoltage(Voltage volts)
  {
    return run(() -> {
      Logger.recordOutput("Hopper/Voltage", volts);
      motor.setVoltage(volts);
    }).withName("HopperSetVoltage");
  }

  public Command set(double dutyCycle)
  {
    return run(() -> {
      Logger.recordOutput("Hopper/DutyCycle", dutyCycle);
      motor.setDutyCycle(dutyCycle);
    }).withName("HopperSetDutyCycle");
  }

  public Command setDutyCycle(Supplier<Double> dutyCycle)
  {
    return run(() -> {
      Logger.recordOutput("Hopper/DutyCycle", dutyCycle.get());
      motor.setDutyCycle(dutyCycle.get());
    }).withName("HopperSetDutyCycleSupplier");
  }

  @Override
  public void simulationPeriodic()
  {
    motor.simIterate();
  }

  @Override
  public void periodic()
  {
    updateInputs();
    //Logger.processInputs("Hopper", hopperInputs);
    motor.updateTelemetry();
  }
}
