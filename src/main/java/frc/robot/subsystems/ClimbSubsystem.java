package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Millimeter;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volt;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortMap;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class ClimbSubsystem extends SubsystemBase {

    private SmartMotorControllerConfig climbMotorConfig = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.OPEN_LOOP)
    .withMechanismCircumference(Meters.of(Meters.convertFrom(0.5, Inches)))
    .withClosedLoopController(4,0,0,MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5))
    .withSimClosedLoopController(4,0,0,MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5))
    .withFeedforward(new ElevatorFeedforward(0, 0, 0))
    .withSimFeedforward(new ElevatorFeedforward(0, 0, 0))
    .withTelemetry("ElevatorMotorController", TelemetryVerbosity.HIGH)
    .withGearing(new MechanismGearing(GearBox.fromReductionStages(5,4,3)))
    .withMotorInverted(false)
    .withIdleMode(MotorMode.COAST)
    .withStatorCurrentLimit(Amps.of(40))
    .withClosedLoopRampRate(Seconds.of(0.5))
    .withOpenLoopRampRate(Seconds.of(0.5))
    .withFollowers(
        new Pair<Object, Boolean>
        (
            new SparkMax(PortMap.ELEVATOR_FOLLOWER_MOTOR_ID, MotorType.kBrushless),
            true
        )
    );

    private SparkMax climbMotor = new SparkMax(41, MotorType.kBrushless);

    private SmartMotorController climbController = new SparkWrapper(climbMotor, DCMotor.getNEO(1), climbMotorConfig);

    private ElevatorConfig elevatorConfig = new ElevatorConfig(climbController)
    .withStartingHeight(Meters.of(0.12))
    .withHardLimits(Meters.of(0), Meters.of(0.12))
    .withTelemetry("Elevator", TelemetryVerbosity.HIGH)
    .withMass(Kilogram.of(12));

    private Elevator elevator = new Elevator(elevatorConfig);

    public ClimbSubsystem() {
    }

    @Override
    public void periodic() {
        elevator.updateTelemetry();
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        elevator.simIterate();
        // This method will be called once per scheduler run during simulation
    }

    public Command setHeight(Distance height) 
    {
        return elevator.runTo(height, Millimeter.of(0.5));
    }

    public Command adjustHeight(Distance addedHeight) 
    {
        return runOnce(() -> 
        {
            elevator.setMeasurementPositionSetpoint(elevator.getHeight().plus(addedHeight));
        }
        );
    }

    public Command setVoltage(double voltage) 
    {
        return run(() -> elevator.setVoltage(Volt.of(voltage)));
    }
}
