// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ChuteSubsystem extends SubsystemBase {

  private Servo leftActuator = new Servo(1);
  private Servo rightActuator = new Servo(2);

  /** Creates a new ChuteSubsystem. */
  public ChuteSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void set(double position) 
  {
    leftActuator.set(position);
    rightActuator.set(position);
  }

  public void setAngle(double degrees) 
  {
    leftActuator.setAngle(degrees);
    rightActuator.setAngle(degrees);
  }
}
