// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PathFinder extends SubsystemBase {

  public enum AutoPosition {
    RightBumpMiddleIntake(new Pose2d(7.459, 1.637, Rotation2d.fromDegrees(82.694)));
    
    Pose2d pose;
    AutoPosition(Pose2d pose2d) {
      this.pose = pose2d;
    }

    public Pose2d getPose() {
      return pose;
    }
  }

  SwerveSubsystem drivebase;
  PathConstraints constraints = new PathConstraints(
        2.0, 3.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));

  public PathFinder(SwerveSubsystem drivebase) {
    this.drivebase = drivebase;


  }

  public Command pathToPositionCommand(AutoPosition targePosition) {
    return AutoBuilder.pathfindToPose(
        targePosition.getPose(),
        constraints,
        0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
