package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pounds;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ElevatorConfig;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class ClimbSubsystem extends SubsystemBase {
    private final SmartMotorControllerConfig config = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withMechanismCircumference(Meters.of(Inches.of(0.25).in(Meters) * 22))
        .withIdleMode(MotorMode.BRAKE)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
        .withMomentOfInertia(Inches.of(0.5), Pounds.of(2))
        .withClosedLoopController(4, 0, 0, MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5))
        .withSimClosedLoopController(4, 0, 0, MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5))
        .withFeedforward(new SimpleMotorFeedforward(0, 0 ,0))
        .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
        .withMotorInverted(false)
        .withTelemetry("ClimbMotor", TelemetryVerbosity.HIGH);

    private final SmartMotorController leftMotor = new SparkWrapper(
        new SparkMax(10, MotorType.kBrushless), DCMotor.getNEO(1), config);

    private final SmartMotorController rightMotor = new SparkWrapper(
        new SparkMax(11, MotorType.kBrushless), DCMotor.getNEO(1), config);

    ElevatorConfig elevConfig = new ElevatorConfig(leftMotor)
        .withStartingHeight(Meters.of(0.5))
        .withHardLimits(Meters.of(0), Meters.of(2.0))
        .withTelemetry("Climbing", TelemetryVerbosity.HIGH)
        .withMass(Pounds.of(10));
}
