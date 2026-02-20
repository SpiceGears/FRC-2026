// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
    public static final boolean USE_VISION = false;
  }

  public static final class VisionConstants
  {
    public static final String SHOOTER_LL4_NAME = "shooterLL4";
    public static final int SHOOTER_LL4_PIPELINE_INDEX = 0;
    
    public static final Pose3d SHOOTER_LL4_CAMERA_OFFSET = new Pose3d(
      Centimeter.of(0),
      Centimeter.of(0),
      Centimeter.of(0),
      
      new Rotation3d(
        Radian.of(0),
        Radian.of(0),
        Radian.of(0)
      )
    );
  }

  public static final class ShooterConstats 
  {
    public static final AngularVelocity INITIAL_TARGET_VELOCITY = RPM.of(5600);
    public static final AngularVelocity VELOCITY_TOLERANCE = RPM.of(50);
    public static final boolean AUTO_PASSTHROUGH_ON_TARGET_VELOCITY = true;
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }




  public static class PortMap 
  {
    public static final int INTAKE_EXTENDER_ID = 22;
    public static final int INTAKE_EXTENDER_FOLLOWER_ID = 23;

    public static final int INTAKE_SPINNER_ID = 21;
    public static final int FEEDER_MOTOR_ID = 24;

    public static final int SHOOTER_LEFT_MOTOR_ID = 31;
    public static final int SHOOTER_MID_MOTOR_ID = 32;
    public static final int SHOOTER_RIGHT_MOTOR_ID = 33;

    public static final int SHOOTER_PASSER_MOTOR_ID = 34;

    public static final int ELEVATOR_MOTOR_ID = 41;
    public static final int ELEVATOR_FOLLOWER_MOTOR_ID = 42;


    public static final int HOOD_LEFT_ACTUATOR_ANALOG_PORT = 1;
    public static final int HOOD_RIGHT_ACTUATOR_ANALOG_PORT = 2;

  }
}

