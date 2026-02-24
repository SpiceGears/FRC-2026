package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeRollersSubsystem;
import frc.robot.subsystems.IntakeRollersSubsystem.RollerSpeed;

public class IntakeCommands {

    // ==========================================
    // KOMENDA AGITATE (Szarpanie i pobieranie)
    // ==========================================
    public static Command agitate(IntakePivotSubsystem pivot, IntakeRollersSubsystem rollers) {
        return Commands.parallel(
            // 1. Włącz rolki na pełną moc pobierania
            rollers.runRollersCommand(RollerSpeed.INTAKE),
            
            // 2. Szarp ramieniem góra-dół
            Commands.sequence(
                pivot.moveUpTimeCommand(0.3), // Ramię w górę przez pół sekundy
                pivot.deployCommand()         // Ramię w dół aż zaryje o zderzak
            ).repeatedly()
            
        ).finallyDo(() -> {
            // PO PUSZCZENIU PRZYCISKU: Wymuś powrót ramienia na dół
            pivot.deployCommand().schedule();
        }).withName("IntakeCommands.Agitate");
    }

    // ==========================================
    // ZWYKŁY INTAKE (Jeśli chcesz tylko pobierać na dole)
    // ==========================================
    public static Command standardIntake(IntakePivotSubsystem pivot, IntakeRollersSubsystem rollers) {
        return Commands.parallel(
            rollers.runRollersCommand(RollerSpeed.INTAKE),
            // Profilaktycznie upewnij się, że ramię jest dociśnięte do dołu
            pivot.deployCommand() 
        ).withName("IntakeCommands.Standard");
    }
}