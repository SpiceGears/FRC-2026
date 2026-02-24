// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.vision.ShooterVisionAid;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoHoodAdjustment extends Command {
  /** Creates a new AutoHoodAdjustment. */
  final HoodSubsystem hood;
  final ShooterVisionAid svas;
  public AutoHoodAdjustment(HoodSubsystem hood) {
    this.hood = hood;
    svas = ShooterVisionAid.instance;
    addRequirements(hood);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    Distance hoodPosition = svas.getHoodPosition(svas.getCurrentOrCachedKey());

    hood.setDistance(hoodPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
