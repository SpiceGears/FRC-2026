package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PortMap;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.gearing.Sprocket;
//import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import yams.mechanisms.SmartMechanism;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class IntakeSubsystem extends SubsystemBase {
    private PWMSparkMax intakeMaster;

    private SparkMax intakeExtenderMotor = new SparkMax(Constants.PortMap.INTAKE_EXTENDER_ID, MotorType.kBrushless);


    private SmartMotorControllerConfig intakeExtenderConfig = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.CLOSED_LOOP)
    .withClosedLoopController(50, 0, 0, 
    DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
    .withSimClosedLoopController(50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
    .withFeedforward(new ArmFeedforward(0, 0, 0))
    .withSimFeedforward(new ArmFeedforward(0, 0, 0))
    .withStatorCurrentLimit(Current.ofBaseUnits(20, Amp))
    .withTelemetry("IntakeExtender_MotorController", TelemetryVerbosity.HIGH)
    .withMotorInverted(false)
    .withGearing(new MechanismGearing(GearBox.fromStages("2:1")))
    .withIdleMode(MotorMode.BRAKE)
    .withClosedLoopRampRate(Seconds.of(0.25))
    .withOpenLoopRampRate(Seconds.of(0.25))
    .withFollowers(new Pair<>(
        new SparkMax(PortMap.INTAKE_EXTENDER_FOLLOWER_ID, MotorType.kBrushless),
         true));
    
    
    private final SmartMotorController intakeExtenderController =
     new SparkWrapper(intakeExtenderMotor, 
     DCMotor.getNeo550(1), 
     intakeExtenderConfig);

    private final ArmConfig intakeExtenderMechanismConfig = new ArmConfig(intakeExtenderController)
    //.withSoftLimits(Degrees.of(-90), Degrees.of(90))
    //.withHardLimit(Degrees.of(-91), Degrees.of(91))
    .withStartingPosition(Degrees.of(-90))
    .withLength(Meters.of(0.5)).withMass(Kilogram.of(1.2))
    .withTelemetry("IntakeExtender", TelemetryVerbosity.HIGH);

    private Arm intakeExtender = new Arm(intakeExtenderMechanismConfig);

    public IntakeSubsystem() {
        intakeMaster = new PWMSparkMax(0);


        SparkMaxConfig intakeMasterConfig = new SparkMaxConfig();

        intakeMasterConfig.voltageCompensation(12)
        .smartCurrentLimit(20)
        .idleMode(IdleMode.kBrake)
        .inverted(true);

        //intakeMaster.configure(intakeMasterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setIntakePower(double power) {
        intakeMaster.set(power);
    }

    public Angle getAngle() 
    {
        return intakeExtender.getAngle();
    }

    public void stopIntake() {
        intakeMaster.stopMotor();
    }

    public double getIntakePower() {
        return intakeMaster.get();
    }

    public void setIntakeVolts(double volts) {
        intakeMaster.setVoltage(volts);
    }

    public Command setAngle(Angle angle) { return intakeExtender.run(angle);}

    public Command setAngleAndStop(Angle angle) { return intakeExtender.runTo(angle, Degrees.of(0));}

    @Override
    public void periodic() 
    {
        intakeExtender.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        intakeExtender.simIterate();
    }
}
