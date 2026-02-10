// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodSubsystem extends SubsystemBase {

  private Servo leftActuator = new Servo(1);
  private Servo rightActuator = new Servo(2);

  /** Creates a new ChuteSubsystem. */
  public HoodSubsystem() 
  {
    leftActuator.setBoundsMicroseconds(2000, 1501, 1500, 1499, 1000);
    rightActuator.setBoundsMicroseconds(2000, 1501, 1500, 1499, 1000);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void set(double position) 
  {
    leftActuator.set(position);
    rightActuator.set(position);
  }

  public void setLenghtMM(double mm) 
  {
    if (mm < 0) mm = 0;
    if (mm > 50) mm = 50;
    leftActuator.set(mm / 50.0);
    rightActuator.set(mm / 50.0);
  }

  public void extendFull() 
  {
    leftActuator.set(1);
    rightActuator.set(1);
  }

  public void retractFull() 
  {
    leftActuator.set(0);
    rightActuator.set(0);
  }
}
