package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortMap;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class IntakePivotSubsystem extends SubsystemBase {

    private final SmartMotorController pivotController;

    public IntakePivotSubsystem() {
        SparkMax pivotMotor = new SparkMax(PortMap.INTAKE_EXTENDER_ID, MotorType.kBrushless);

        SmartMotorControllerConfig config = new SmartMotorControllerConfig(this)
            // ZMIANA: Czysty Open Loop, żadnego PID i feedforwardu
            .withControlMode(ControlMode.OPEN_LOOP) 
            .withStatorCurrentLimit(Current.ofBaseUnits(20, Amp))
            .withTelemetry("IntakePivot", TelemetryVerbosity.HIGH)
            .withMotorInverted(true) // Zakładamy ujemne napięcie w dół
            .withGearing(new MechanismGearing(GearBox.fromStages("4:1", "4:1", "5:1", "34:16")))
            .withIdleMode(MotorMode.BRAKE)
            .withOpenLoopRampRate(Seconds.of(0.25))
            .withSoftLimit(Degrees.of(-5), Degrees.of(100))
            .withStartingPosition(Degrees.of(90))
            .withFollowers(new Pair<>(new SparkMax(PortMap.INTAKE_EXTENDER_FOLLOWER_ID, MotorType.kBrushless), false));
        
        pivotController = new SparkWrapper(pivotMotor, DCMotor.getNeo550(1), config);
    }

    public void setVoltage(double volts) {
        pivotController.setVoltage(Volts.of(volts));
    }

    public Command deployCommand() {
        return Commands.run(() -> setVoltage(-1.5), this)
            // ZMIANA: Zamiast prądu, sprawdzamy czy ramię zjechało do soft limitu.
            // Zakładam, że dolny soft limit to około -5 stopni, więc przerywamy poniżej -4.5
            .until(() -> pivotController.getMechanismPosition().in(Degrees) <= -4) 
            .andThen(Commands.runOnce(() -> setVoltage(0), this))
            .withName("Pivot.Deploy");
    }

    public Command moveUpTimeCommand(double timeSeconds) {
        return Commands.run(() -> setVoltage(3.0), this) // Jedź w górę (+3.0V)
            .withTimeout(timeSeconds) // Przez X sekund
            .andThen(Commands.runOnce(() -> setVoltage(0), this)) // Zatrzymaj silnik
            .withName("Pivot.MoveUp");
    }
}