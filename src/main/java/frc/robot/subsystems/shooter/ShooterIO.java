package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkBase.Faults;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  public static class ShooterIOInputs {
    public double shooterAmperage = 0.0;
    public double shooterVoltage = 0.0;

    public Faults shooterFaults = new Faults(0);
    public double shooterVelocity = 0.0;

    public boolean shooterTargetVelocityReached = false;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void startShooter() {}

  public default void setShooterTargetVelocity(double velocity) {}

  public default void stopShooter() {}
}
