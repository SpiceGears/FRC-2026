package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortMap;

public class HoodSubsystem extends SubsystemBase {

  private final Servo leftActuator = new Servo(1);
  private final Servo rightActuator = new Servo(2);

  private final SparkMax hoodMotor = new SparkMax(PortMap.HOOD_MOTOR_ID, MotorType.kBrushless);

  private static final double SAFE_MIN_MM = 5.0; 
  private static final double SAFE_MAX_MM = 28.0; 
  private static final double MAX_STROKE_MM = 50.0;
  private static final double SPEED_PER_SECOND = 8.0; // [mm]
  private static final double TOLERANCE = 0.5; // [mm]

  private final InterpolatingDoubleTreeMap hoodPositionEncoderMap = new InterpolatingDoubleTreeMap();


  

  private double currentTargetMm = 0.0;

  private double currentPosition = 0.0;

  private double lastLoopTime;
  private double currentLoopTime;

  public HoodSubsystem() {
    leftActuator.setBoundsMicroseconds(2000, 1501, 1500, 1499, 1000);
    rightActuator.setBoundsMicroseconds(2000, 1501, 1500, 1499, 1000);

    SparkMaxConfig hoodConfig = new SparkMaxConfig();
    hoodConfig.inverted(true)
    .closedLoop
    .p(0.1)
    .i(0.0)
    .d(0.0)
    .minOutput(-0.5)
    .maxOutput(0.5)
    .feedForward
    .apply(new FeedForwardConfig().kS(0.0).kV(0.0));

    hoodConfig.smartCurrentLimit(20)
    .idleMode(IdleMode.kBrake)
    .closedLoopRampRate(0.25)
    .voltageCompensation(12);

    hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    hoodMotor.getEncoder().setPosition(0.0);

    initializeMap();

    lastLoopTime = Timer.getFPGATimestamp();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hood/Target Position (mm)", currentTargetMm);
    SmartDashboard.putBoolean("Hood/Is Ready", isReady());
    SmartDashboard.putNumber("Hood/Current encoder position", hoodMotor.getEncoder().getPosition());

    // currentLoopTime = Timer.getFPGATimestamp();
    // double delta = currentLoopTime - lastLoopTime;
    // lastLoopTime = currentLoopTime;

    // double distance = currentTargetMm - currentPosition;
    // double distancePerLoop = SPEED_PER_SECOND * delta;

    // currentPosition += Math.signum(distance) * distancePerLoop;
  }

  public void setTarget(double encoderTarget) 
  {
    hoodMotor.getClosedLoopController()
    .setSetpoint(encoderTarget, ControlType.kPosition);
  }

  public void setExtensionMm(double targetMm) {
    currentTargetMm = MathUtil.clamp(targetMm, SAFE_MIN_MM, SAFE_MAX_MM);

    // double servoPosition = currentTargetMm / MAX_STROKE_MM;

    // leftActuator.set(servoPosition);
    // rightActuator.set(servoPosition);

    double encoderTarget = hoodPositionEncoderMap.get(currentTargetMm);
    setTarget(encoderTarget);
  }

  public boolean isReady() 
  {
    return Math.abs(hoodMotor.getEncoder().getPosition() - hoodPositionEncoderMap.get(currentTargetMm)) <= TOLERANCE;
  }

  public double getTargetExtensionMm() {
    return currentTargetMm;
  }





  public void initializeMap() 
  {
    hoodPositionEncoderMap.put(SAFE_MIN_MM, 0.0);
    hoodPositionEncoderMap.put(23.0, 13.6909);
  }
}