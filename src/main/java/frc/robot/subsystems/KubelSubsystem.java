package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil; // Potrzebne do interpolate
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class KubelSubsystem extends SubsystemBase {

  // --- KONFIGURACJA ---
  // Kąty i dystanse (w radianach i metrach)
  private final double intakeOpenAngle = 0;
  private final double intakeClosedAngle = 1.5;
  private final double hopperOpenDistance = -0.55;
  private final double hopperClosedDistance = -0.3;

  // Prędkość symulacji (0.01 = wolno, 0.1 = szybko, 1.0 = natychmiast)
  // 0.05 oznacza, że w każdym cyklu (20ms) pokonujemy 5% pozostałego dystansu
  private final double SIM_SPEED = 0.08; 

  // --- ZMIENNE STANU ---
  
  // Gdzie mechanizm JEST w danej chwili (do wizualizacji)
  private double actualIntakeAngle = intakeClosedAngle;
  private double actualHopperDistance = hopperClosedDistance;

  // Gdzie mechanizm CHCE BYĆ (cel ustawiany przez komendy)
  private double targetIntakeAngle = intakeClosedAngle;
  private double targetHopperDistance = hopperClosedDistance;

  public KubelSubsystem() {
    // Konstruktor
  }

  @Override
  public void periodic() {
    // 1. LOGIKA SYMULACJI RUCHU
    // W każdym cyklu (20ms) przesuwamy 'actual' w stronę 'target'
    // MathUtil.interpolate działa jak płynne wygładzanie ruchu
    actualIntakeAngle = MathUtil.interpolate(actualIntakeAngle, targetIntakeAngle, SIM_SPEED);
    actualHopperDistance = MathUtil.interpolate(actualHopperDistance, targetHopperDistance, SIM_SPEED);

    // 2. WIZUALIZACJA (LOGOWANIE)
    // Logujemy pozycje bazowe (jeśli są potrzebne w AdvantageScope jako rodzic)
    Logger.recordOutput("ZeroedIntake", new Pose3d());
    Logger.recordOutput("ZeroedHopper", new Pose3d());

    // Logujemy aktualne (ruchome) pozycje
    // Używamy zmiennych 'actual...', żeby widzieć płynny ruch
    Logger.recordOutput("FinalIntake", new Pose3d(
        -0.26, 0, 0.16, 
        new Rotation3d(0.0, actualIntakeAngle, 0.0)
    ));
    
    Logger.recordOutput("FinalHopper", new Pose3d(
        actualHopperDistance, 0, 0.285, 
        new Rotation3d(0.0, 0.0, 0.0)
    ));
  }

  // --- KOMENDY PODSTAWOWE (Zmieniają tylko CEL) ---

  Command openIntake() {
    // Zmieniamy cel, a periodic zajmie się ruchem
    return this.runOnce(() -> targetIntakeAngle = intakeOpenAngle);
  }

  Command closeIntake() {
    return this.runOnce(() -> targetIntakeAngle = intakeClosedAngle);
  }

  Command openHopper() {
    return this.runOnce(() -> targetHopperDistance = hopperOpenDistance);
  }

  Command closeHopper() {
    return this.runOnce(() -> targetHopperDistance = hopperClosedDistance);
  }

  // --- KOMENDY ZŁOŻONE (SEKWENCJE) ---

  public Command openKubel() {
    return new SequentialCommandGroup(
      openHopper(),        // 1. Ustaw cel Hoppera na otwarty (zaczyna się ruch w periodic)
      new WaitCommand(0.15), // 2. Czekaj 0.5s aż Hopper wizualnie dojedzie
      openIntake()         // 3. Dopiero wtedy otwórz Intake
    );
  }

  public Command closeKubel() {
    return new SequentialCommandGroup(
      closeIntake(),        // 1. Zamknij Intake
      new WaitCommand(0.15), // 2. Czekaj aż dojedzie
      closeHopper()         // 3. Zamknij Hopper
    );
  }
}