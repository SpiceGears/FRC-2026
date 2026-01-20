package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class ShooterSubsystem extends SubsystemBase {
    private SmartMotorControllerConfig rightWheelConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withIdleMode(MotorMode.COAST)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
        .withMomentOfInertia(Inches.of(4), Pounds.of(2))
        .withClosedLoopController(1, 0, 0)
        .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
        .withMotorInverted(false)
        .withTelemetry("RightFlyWheel", SmartMotorControllerConfig.TelemetryVerbosity.HIGH);

    private SmartMotorController rightWheel = new SparkWrapper(
        new SparkMax(4, MotorType.kBrushless), DCMotor.getNEO(1), rightWheelConfig);

    private SmartMotorControllerConfig leftWheelConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withIdleMode(MotorMode.COAST)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
        .withMomentOfInertia(Inches.of(4), Pounds.of(2))
        .withClosedLoopController(1, 0, 0)
        .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
        .withMotorInverted(false)
        .withTelemetry("LeftFlyWheel", SmartMotorControllerConfig.TelemetryVerbosity.HIGH);

    private SmartMotorController leftWheel = new SparkWrapper(
        new SparkMax(5, MotorType.kBrushless),  DCMotor.getNEO(1), leftWheelConfig);

    public ShooterSubsystem() {
        rightWheel.setupTelemetry();
        leftWheel.setupTelemetry();
    }

    public Pair<LinearVelocity, LinearVelocity> getLinearSpeed() {
        return Pair.of(rightWheel.getMeasurementVelocity(), leftWheel.getMeasurementVelocity());
    }


}
