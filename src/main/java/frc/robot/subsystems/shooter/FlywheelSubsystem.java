// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volt;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PortMap;
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

public class FlywheelSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  SmartMotorControllerConfig shooterMotorConfig = new SmartMotorControllerConfig(this)
  .withControlMode(ControlMode.CLOSED_LOOP)
  .withClosedLoopController(
    0.02, 0, 0
    // RPM.of(6000), DegreesPerSecondPerSecond.of(720)
    )
  .withFeedforward(new SimpleMotorFeedforward(0.05, 0.11, 0.05)
  ).withSimFeedforward(new SimpleMotorFeedforward(0, 0))
  .withTelemetry("ShooterMotor", TelemetryVerbosity.MID)
  .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
  .withMotorInverted(false)
  .withIdleMode(MotorMode.COAST)
  .withStatorCurrentLimit(Amps.of(40))
  .withFollowers(
    new Pair<Object, Boolean>(
      new SparkMax(PortMap.SHOOTER_MID_MOTOR_ID, MotorType.kBrushless),
      false
    ),
    new Pair<Object, Boolean>(
      new SparkMax(PortMap.SHOOTER_RIGHT_MOTOR_ID, MotorType.kBrushless),
      true
     )
    );

  private final SparkMax shooterSpark = new SparkMax(Constants.PortMap.SHOOTER_LEFT_MOTOR_ID, MotorType.kBrushless);


  private SmartMotorController shooterController = new SparkWrapper(shooterSpark, DCMotor.getNEO(1), shooterMotorConfig);

  private final FlyWheelConfig shooterMechanismConfig = new FlyWheelConfig(shooterController)
  .withDiameter(Centimeter.of(10))
  .withMass(Kilograms.of(1.5))
  .withUpperSoftLimit(RPM.of(6000))
  .withLowerSoftLimit(RPM.of(0))
  .withTelemetry("ShooterMechanism", TelemetryVerbosity.HIGH);
  

  private FlyWheel shooter = new FlyWheel(shooterMechanismConfig);

  private boolean shooterEnabled = false;

  private AngularVelocity targetVelocity = Constants.ShooterConstats.INITIAL_TARGET_VELOCITY;

  public void setTargetVelocity(AngularVelocity target) { this.targetVelocity = target;}

  public FlywheelSubsystem() {}

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


  public AngularVelocity getVelocity() {
    return shooter.getSpeed();
  }

  public boolean isAtTargetVelocity() 
  {
    return shooter.getSpeed().isNear(targetVelocity, Constants.ShooterConstats.VELOCITY_TOLERANCE);
  }

  public Command setVelocity(AngularVelocity velocity) 
  {
    return shooter.setSpeed(velocity);
  }



  public void setVelocitySetpoint(AngularVelocity setpoint) 
  {
    // shooter.getMotorController().startClosedLoopController();
    shooter.setMechanismVelocitySetpoint(setpoint);
  }

  public void setVoltage(Voltage volts) 
  {
    shooter.setVoltageSetpoint(volts);
  }

  public void stopControl() 
  {
    // shooter.getMotorController().setContro();
    setVoltage(Volt.of(0));
  }

  public void spinUpToVelocity(AngularVelocity setpoint) 
  {
    shooter.getMotorController().startClosedLoopController();
    shooter.setMechanismVelocitySetpoint(setpoint);
  }

}