/// Mockup LL4 vision subsystem

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightTarget_Detector;

public class LimelightDetector extends SubsystemBase {
  
  LimelightTarget_Detector limel;
  
  
  /** Creates a new LimelightDetector. */
  
  
  public LimelightDetector() 
  {
    limel = new LimelightTarget_Detector();
    LimelightHelpers.setPipelineIndex("", 1);
    Pose3d fuel = LimelightHelpers.getTargetPose3d_RobotSpace("");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void test()
  {
    
  }
}
