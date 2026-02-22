package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PortMap;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
//import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity
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

    final double INTAKE_SPEED = 1.0;

    private SparkMax intakeExtenderMotor = new SparkMax(Constants.PortMap.INTAKE_EXTENDER_ID, MotorType.kBrushless);


    private SmartMotorControllerConfig intakeExtenderConfig = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.CLOSED_LOOP)
    .withClosedLoopController(1, 0, 0, 
    DegreesPerSecond.of(720), DegreesPerSecondPerSecond.of(720))
    .withSimClosedLoopController(5, 0, 0, DegreesPerSecond.of(9000), DegreesPerSecondPerSecond.of(45))
    .withFeedforward(new ArmFeedforward(0.65, 0.0, 0))
    // .withSimFeedforward(new ArmFeedforward(0, 0, 0))
    .withStatorCurrentLimit(Current.ofBaseUnits(20, Amp))
    .withTelemetry("IntakeExtender_MotorController", TelemetryVerbosity.HIGH)
    .withMotorInverted(false)
    .withGearing(new MechanismGearing(GearBox.fromStages("4:1","4:1","5:1", "34:16")))
    .withIdleMode(MotorMode.BRAKE)
    .withClosedLoopRampRate(Seconds.of(0.25))
    .withOpenLoopRampRate(Seconds.of(0.25))
    //.withSoftLimit(Degrees.of(-5), Degrees.of(95))
    .withFollowers(new Pair<>(
        new SparkMax(PortMap.INTAKE_EXTENDER_FOLLOWER_ID, MotorType.kBrushless),
         true));
    
    
    private final SmartMotorController intakeExtenderController =
     new SparkWrapper(intakeExtenderMotor, 
     DCMotor.getNeo550(1), 
     intakeExtenderConfig);

    private final ArmConfig intakeExtenderMechanismConfig = new ArmConfig(intakeExtenderController)
    //.withSoftLimits(Degrees.of(-5), Degrees.of(95))
    //.withHardLimit(Degrees.of(-10), Degrees.of(100))
    .withStartingPosition(Degrees.of(90))
    .withLength(Meters.of(0.5)).withMass(Kilogram.of(1.2))
    .withTelemetry("IntakeExtender", TelemetryVerbosity.HIGH);

    private Arm intakeExtender = new Arm(intakeExtenderMechanismConfig);

    public IntakeSubsystem() {
        intakeMaster = new PWMSparkMax(0);


        SparkMaxConfig intakeMasterConfig = new SparkMaxConfig();

        intakeMasterConfig.voltageCompensation(12)
        .smartCurrentLimit(20)
        .idleMode(IdleMode.kBrake)
        .inverted(false);

        //intakeMaster.configure(intakeMasterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setIntakePower(double power) {
        intakeMaster.set(power * -1);
    }

    public Angle getAngle() 
    {
        return intakeExtender.getAngle();
    }

    public void stopIntake() {
        intakeMaster.stopMotor();
    }

    public Command runIntakeCommand(double powerMultiplier) 
    {
        return Commands.runEnd(() -> 
        {
            setIntakePower(INTAKE_SPEED * powerMultiplier);
        },
        () -> { stopIntake(); },
        this)
        .withName("Intake.RunRollers");
    }

    public double getIntakePower() {
        return intakeMaster.get();
    }

    public void setIntakeVolts(double volts) {
        intakeMaster.setVoltage(volts);
    }

    public Command setAngleCmd(Angle angle) { return intakeExtender.run(angle);}

    public Command setAngleAndStopCmd(Angle angle) { return intakeExtender.runTo(angle, Degrees.of(0));}

    public void setAngle(Angle angle) {intakeExtender.setMechanismPositionSetpoint(angle);}

    @Override
    public void periodic() 
    {
        intakeExtender.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        intakeExtender.simIterate();
    }

    public Command adjustIntake(Angle adjustment) 
    {
      return runOnce(() -> 
      {
        setAngle(this.getAngle().plus(adjustment));
      });
    }
}
