// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision.limelight;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import limelight.Limelight;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.PoseEstimate;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.LimelightSettings.ImuMode;
import limelight.networktables.LimelightSettings.LEDMode;
import limelight.networktables.LimelightSettings.StreamMode;

public class AprilTagVisionSubsystem extends SubsystemBase {
  /** Creates a new AprilTagVisionSubsystem. */


  // Use singleton pattern here
  public static AprilTagVisionSubsystem instance;

  public static Field2d field = new Field2d();

  Limelight shooterCamera = new Limelight(VisionConstants.SHOOTER_LL4_NAME);
  LimelightPoseEstimator poseEstimator = new LimelightPoseEstimator(shooterCamera, EstimationMode.MEGATAG1);

  public AprilTagVisionSubsystem() 
  {
    if (instance != null) {
      //throw new IllegalStateException("AprilTagVisionSubsystem instance already exists!");
      return;
    }

    shooterCamera.getSettings()
    .withCameraOffset(VisionConstants.SHOOTER_LL4_CAMERA_OFFSET)
    .withLimelightLEDMode(LEDMode.PipelineControl)
    .withPipelineIndex(VisionConstants.SHOOTER_LL4_PIPELINE_INDEX)
    .withStreamMode(StreamMode.Standard)
    .withImuMode(ImuMode.InternalImu)
    .save();

    instance = this;
  }

  public Optional<Pose3d> getEstimatedPose() {
    Optional<PoseEstimate> pose = poseEstimator.getPoseEstimate();
    if (pose.isEmpty()) {
      return Optional.empty();
    }

    return Optional.of(pose.get().pose);
  }

  public Optional<Pose2d> getEstimatedPose2d() {

    Optional<Pose3d> estimatedPose3d = getEstimatedPose();
    if (estimatedPose3d.isEmpty()) {
      return Optional.empty();
    }

    return estimatedPose3d.map(p -> new Pose2d(p.getTranslation().toTranslation2d(), p.getRotation().toRotation2d()));
  }

  @Override
  public void periodic() {
    //SmartDashboard.putData("AprilTagVisionSubsystem/Estimated Pose2D", instance.getEstimatedPose2d());
    Optional<Pose2d> estimatedPose2d = instance.getEstimatedPose2d();
    if (estimatedPose2d.isPresent()) {
    field.setRobotPose(estimatedPose2d.get());
    SmartDashboard.putData("Vision Estimated pose",field);
    }
    // This method will be called once per scheduler run
  }
}
