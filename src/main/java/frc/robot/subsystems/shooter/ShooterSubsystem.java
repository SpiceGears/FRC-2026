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


  public final FlywheelSubsystem flywheel;
  public final PasserSubsystem passer;
  public final HoodSubsystem hood;

  double currentFlywheelKey = 1.0;
  double currentHoodKey = 1.0;

  boolean enabled = false;


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

  public Command setFlywheelParametersCommand(double key) 
  {
    return runOnce(() -> 
    {
      this.setFlywheelParameter(key);
    });
  }

  public void applyParameters() 
  {
    applyFlywheelParameter();
    applyHoodParameter();
    // return Commands.parallel(
    //   flywheelCmd,
    //   hoodCmd
    // );
  }

  public void applyFlywheelParameter() 
  {
    double flywheelRPM = flywheelRPMMap.get(currentFlywheelKey);
    flywheel.setVelocity(RPM.of(flywheelRPM));
  }

  public void applyHoodParameter() 
  {
    double hoodPosition = hoodPositionMap.get(currentHoodKey);
    hood.setLenghtMM(hoodPosition);
  }

  public Command startCmd() 
  {
    // return setFlywheelParametersCommand(0.95).andThen(applyParameters());
    return runOnce(() -> 
    {
      // this.enabled = true;
      // setFlywheelParameter(0.95);
      // applyParameters();
      start();
    });
  }

  public void start() 
  {
    this.enabled = true;
    setFlywheelParameter(0.95);
    applyParameters();
  }

  public void stop() 
  {
    this.enabled = false;
    setFlywheelParameter(0.0);
    applyParameters();
  }

  public Command stopCmd() 
  {

    // return setFlywheelParametersCommand(0).andThen(applyParameters());
    return runOnce(() -> 
    {
      // this.enabled = false;
      // setFlywheelParameter(0.0);
      // applyParameters();
      stop();
    });
  }

  public Command toggleEnabledCmd() 
  {
    return runOnce(() -> 
    {
      toggleEnabled();
    });
  }

  public void adjustHood(double addedKey) 
  {
    currentHoodKey += addedKey;
    currentHoodKey = MathUtil.clamp(currentHoodKey, 0, 1);
    setHoodParameter(currentHoodKey);
    applyParameters();
  }

  public void toggleEnabled() 
  {
      if (!enabled) start();
      else stop();
  }

  public boolean isEnabled() 
  {
    return this.enabled;
  }
}
