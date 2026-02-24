package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeRollersSubsystem;
import frc.robot.subsystems.IntakeRollersSubsystem;

public class IntakeFuel extends Command {
    public final IntakeRollersSubsystem intakeSubsystem;

    public IntakeFuel(IntakeRollersSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(this.intakeSubsystem);
    }

    @Override
    public void initialize() {
        //intakeSubsystem.setIntakePower(1.0);
    }

    @Override
    public void end(boolean interrupted) {
        //intakeSubsystem.stopIntake();
    }
}
