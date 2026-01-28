// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.motorcontrol.Koors40;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.local.SparkWrapper;

public class HopperSubsystem extends SubsystemBase {
  private final SmartMotorControllerConfig HopperConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withIdleMode(MotorMode.COAST)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
      .withMomentOfInertia(Inches.of(0), Pounds.of(0))
      .withClosedLoopController(0, 0, 0)
      .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
      .withMotorInverted(false)
      .withTelemetry("HopperMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH);


  private final SmartMotorController HopperController = new SparkWrapper(new SparkMax(0, MotorType.kBrushless), DCMotor.getNEO(2), HopperConfig);







  /** Creates a new HopperSubsystem. */
  public HopperSubsystem() {
    HopperController.setupTelemetry();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
