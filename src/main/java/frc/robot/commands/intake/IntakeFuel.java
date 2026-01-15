package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeFuel extends Command {
    public final IntakeSubsystem intakeSubsystem;

    public IntakeFuel(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(this.intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.setIntakePower(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopIntake();
    }
}
