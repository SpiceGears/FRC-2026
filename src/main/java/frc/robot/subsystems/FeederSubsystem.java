// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortMap;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class FeederSubsystem extends SubsystemBase {

  private SmartMotorControllerConfig feederControllerConfig = new SmartMotorControllerConfig(this)
  .withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP)
  .withClosedLoopController(5,0,0,RPM.of(6000), DegreesPerSecondPerSecond.of(90))
  .withMotorInverted(false)
  .withIdleMode(MotorMode.BRAKE)
  .withStatorCurrentLimit(Amp.of(20))
  .withGearing(1)
  .withTelemetry("FeederMotor", TelemetryVerbosity.LOW);
  private SparkMax feederMotor = new SparkMax(PortMap.FEEDER_MOTOR_ID, MotorType.kBrushed);

  private SmartMotorController feederController = new SparkWrapper(feederMotor, DCMotor.getVex775Pro(1), feederControllerConfig);

  /** Creates a new FeederSubsystem. */
  public FeederSubsystem() 
  {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void feedShooter(double speed) 
  {
    feederController.setVelocity(MetersPerSecond.of(Meter.convertFrom(speed, Centimeter)));
  }

  public void stop() 
  {
    //set desired duty cycle to 0, effectively stopping the mechanism
    feederController.setDutyCycle(0);
  }
}
