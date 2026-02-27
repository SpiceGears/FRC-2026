package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodSubsystem extends SubsystemBase {

  private final Servo leftActuator = new Servo(1);
  private final Servo rightActuator = new Servo(2);

  private static final double SAFE_MIN_MM = 5.0; 
  private static final double SAFE_MAX_MM = 28.0; 
  private static final double MAX_STROKE_MM = 50.0;
  private static final double SPEED_PER_SECOND = 8.0; // [mm]
  private static final double TOLERANCE = 0.5; // [mm]


  

  private double currentTargetMm = 0.0;

  private double currentPosition = 0.0;

  private double lastLoopTime;
  private double currentLoopTime;

  public HoodSubsystem() {
    leftActuator.setBoundsMicroseconds(2000, 1501, 1500, 1499, 1000);
    rightActuator.setBoundsMicroseconds(2000, 1501, 1500, 1499, 1000);

    
    lastLoopTime = Timer.getFPGATimestamp();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hood/Target Position (mm)", currentTargetMm);
    SmartDashboard.putBoolean("Hood/Is Ready", isReady());
    SmartDashboard.putNumber("Hood/Current position", currentPosition);

    currentLoopTime = Timer.getFPGATimestamp();
    double delta = currentLoopTime - lastLoopTime;
    lastLoopTime = currentLoopTime;

    double distance = currentTargetMm - currentPosition;
    double distancePerLoop = SPEED_PER_SECOND * delta;

    currentPosition += Math.signum(distance) * distancePerLoop;
  }

  public void setExtensionMm(double targetMm) {
    currentTargetMm = MathUtil.clamp(targetMm, SAFE_MIN_MM, SAFE_MAX_MM);

    double servoPosition = currentTargetMm / MAX_STROKE_MM;

    leftActuator.set(servoPosition);
    rightActuator.set(servoPosition);
  }

  public boolean isReady() 
  {
    return Math.abs(currentPosition - currentTargetMm) <= TOLERANCE;
  }

  public double getTargetExtensionMm() {
    return currentTargetMm;
  }
}