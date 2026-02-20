// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */


  final FlywheelSubsystem flywheel;
  final PasserSubsystem passer;
  final HoodSubsystem hood;

  double currentFlywheelKey = 1.0;
  double currentHoodKey = 1.0;


  InterpolatingDoubleTreeMap flywheelRPMMap = new InterpolatingDoubleTreeMap();
  InterpolatingDoubleTreeMap hoodPositionMap = new InterpolatingDoubleTreeMap();
  public ShooterSubsystem(FlywheelSubsystem flywheel, PasserSubsystem passer, HoodSubsystem hood) {
    this.flywheel = flywheel;
    this.passer = passer;
    this.hood = hood;

    configureRPMAngleMaps();
  }


  private void configureRPMAngleMaps() 
  {
    flywheelRPMMap.put(0.0, 0.0);
    flywheelRPMMap.put(0.25, 3000.0);
    flywheelRPMMap.put(1.0, 6000.0);

    hoodPositionMap.put(0.0, 0.0);
    hoodPositionMap.put(0.25, 30.0);
    hoodPositionMap.put(1.0, 50.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setFlywheelParameter(double key) 
  {
    currentFlywheelKey = MathUtil.clamp(key, 0.0, 1.0);
  }

  public void setHoodParameter(double key) {
    currentHoodKey = MathUtil.clamp(key, 0.0, 1.0);
  }

  public void setParameters(double key)
  {
    key = MathUtil.clamp(key, 0.0, 1.0);

    currentFlywheelKey = key;
    currentHoodKey = key;
  }

  public Command applyParameters() 
  {
    double flywheelRPM = flywheelRPMMap.get(currentFlywheelKey);
    double hoodPosition = hoodPositionMap.get(currentHoodKey);

    Command flywheelCmd = flywheel.setVelocity(RPM.of(flywheelRPM));
    Command hoodCmd = Commands.run(() -> hood.setLenghtMM(hoodPosition), hood);

    return Commands.parallel(
      flywheelCmd,
      hoodCmd
    );
  }

  public Command start() 
  {
    return applyParameters();
  }

  public Command stop() 
  {
    setFlywheelParameter(0);
    return applyParameters();
  }
}
