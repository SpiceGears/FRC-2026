// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Millimeters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodSubsystem extends SubsystemBase {

  private Servo leftActuator = new Servo(1);
  private Servo rightActuator = new Servo(2);

  private final double MAX_LENGHT_MM = 50;
  private final double MIN_LENGHT_MM = 0;

  private Distance currentPosition = Distance.ofBaseUnits(0, Millimeters);

  /** Creates a new ChuteSubsystem. */
  public HoodSubsystem() 
  {
    leftActuator.setBoundsMicroseconds(2000, 1501, 1500, 1499, 1000);
    rightActuator.setBoundsMicroseconds(2000, 1501, 1500, 1499, 1000);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("HoodCurrentPosition", currentPosition.baseUnitMagnitude());
  }


  /// Set the position of hood; double in range of 0 to 1
  public void set(double position) 
  {
    leftActuator.set(position);
    rightActuator.set(position);
  }

  // go to lenght in mm
  public void setLenghtMM(double mm) 
  {
    if (mm < 0) mm = 0;
    if (mm > 50) mm = 50;
    leftActuator.set(mm / 50.0);
    rightActuator.set(mm / 50.0);
  }

  public void setDistance(Distance distance) 
  {
    currentPosition = distance;
    this.setLenghtMM(distance.in(Millimeters));
  }

  public void setTarget(Distance targetDistance) 
  {
    currentPosition = targetDistance;
  }

  public void goToTarget() 
  {
    this.setDistance(currentPosition);
  }


  /// get the position that is specified as target for actuators
  public Distance getPosition() 
  {
    return currentPosition;
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