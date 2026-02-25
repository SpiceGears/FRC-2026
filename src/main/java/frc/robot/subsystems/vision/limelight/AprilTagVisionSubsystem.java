// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision.limelight;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Rotation;

import java.util.LinkedList;
import java.util.List;
import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightResults;
import limelight.networktables.PoseEstimate;
import swervelib.imu.SwerveIMU;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.LimelightSettings.ImuMode;
import limelight.networktables.LimelightSettings.LEDMode;
import limelight.networktables.LimelightSettings.StreamMode;
import limelight.networktables.target.AprilTagFiducial;
import limelight.networktables.Orientation3d;

public class AprilTagVisionSubsystem extends SubsystemBase {
  /** Creates a new AprilTagVisionSubsystem. */


  // Use singleton pattern here
  public static AprilTagVisionSubsystem instance;

  public static SwerveIMU imu;

  public static Field2d field = new Field2d();

  Limelight shooterCamera = new Limelight(VisionConstants.SHOOTER_LL4_NAME);
  LimelightPoseEstimator poseEstimator;

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
    .withImuMode(ImuMode.SyncInternalImu)
    .save();

    poseEstimator = shooterCamera.createPoseEstimator(EstimationMode.MEGATAG2);


    instance = this;
  }

  public Optional<PoseEstimate> getEstimatedPose() {
    Optional<PoseEstimate> pose = poseEstimator.getPoseEstimate();
    //shooterCamera.getLatestResults().get().targets_Fiducials[0].getTargetPose_RobotSpace();
    if (pose.isEmpty()) {
      return Optional.empty();
    }

    return Optional.of(pose.get());
  }


  public Optional<List<Pair<Double, Pose3d>>> getTargetPoses() 
  {
    Optional<LimelightResults> resultsOpt = shooterCamera.getLatestResults();
    if (resultsOpt.isEmpty()) return Optional.empty();

    LimelightResults results = resultsOpt.get();
    List<Pair<Double,Pose3d>> targetPoses = new LinkedList<>();
    for (AprilTagFiducial target : results.targets_Fiducials) {
      targetPoses.add(new Pair<>(target.fiducialID, target.getTargetPose_RobotSpace())
      );
    }
    return Optional.of(targetPoses);
  }


  // public Optional<Pose2d> getEstimatedPose2d() {

  //   Optional<Pose3d> estimatedPose3d = getEstimatedPose().get().pose;
  //   if (estimatedPose3d.isEmpty()) {
  //     return Optional.empty();
  //   }

  //   return estimatedPose3d.map(p -> new Pose2d(p.getTranslation().toTranslation2d(), p.getRotation().toRotation2d()));
  // }

  @Override
  public void periodic() {

    Pigeon2 pigeon = (Pigeon2) imu.getIMU();
    // Required for megatag2 in periodic() function before fetching pose.
    shooterCamera.getSettings()
        .withRobotOrientation(new Orientation3d(imu.getRotation3d(),
            new AngularVelocity3d(DegreesPerSecond.of(pigeon.getAngularVelocityXDevice().getValueAsDouble()),
                DegreesPerSecond.of(pigeon.getAngularVelocityYDevice().getValueAsDouble()),
                imu.getYawAngularVelocity())))
        .save();
    // SmartDashboard.putData("AprilTagVisionSubsystem/Estimated Pose2D",
    // instance.getEstimatedPose2d());
    Optional<PoseEstimate> poseEst = instance.getEstimatedPose();

    if (poseEst.isPresent()) {

      Pose2d estimatedPose2d = poseEst.get().pose.toPose2d();
      // if (estimatedPose2d.isPresent())
      {
        field.setRobotPose(estimatedPose2d);
        SmartDashboard.putData("Vision/Estimated pose", field);
        SmartDashboard.putNumber("Vision/Estimated Pose Yaw Rotations", Rotation.convertFrom(imu.getRotation3d().getZ(), Degrees));
      }

      
    }
    
    if (getTargetPoses().isPresent()) 
    SmartDashboard.putNumber("Vision/Detection Count", getTargetPoses().get().size());
    else
    SmartDashboard.putNumber("Vision/Detection Count", 0);



    
    // This method will be called once per scheduler run
  }
}
