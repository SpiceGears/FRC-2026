package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodSubsystem extends SubsystemBase {

  private final Servo leftActuator = new Servo(1);
  private final Servo rightActuator = new Servo(2);

  private static final double SAFE_MIN_MM = 5.0; 
  private static final double SAFE_MAX_MM = 28.0; 
  private static final double MAX_STROKE_MM = 50.0;

  private double currentTargetMm = 0.0;

  public HoodSubsystem() {
    leftActuator.setBoundsMicroseconds(2000, 1501, 1500, 1499, 1000);
    rightActuator.setBoundsMicroseconds(2000, 1501, 1500, 1499, 1000);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hood/Target Position (mm)", currentTargetMm);
  }

  public void setExtensionMm(double targetMm) {
    currentTargetMm = MathUtil.clamp(targetMm, SAFE_MIN_MM, SAFE_MAX_MM);

    double servoPosition = currentTargetMm / MAX_STROKE_MM;

    leftActuator.set(servoPosition);
    rightActuator.set(servoPosition);
  }

  public double getTargetExtensionMm() {
    return currentTargetMm;
  }
}