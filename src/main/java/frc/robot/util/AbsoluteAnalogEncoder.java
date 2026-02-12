package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AnalogEncoder;
import org.littletonrobotics.junction.Logger;

public class AbsoluteAnalogEncoder implements AbsoluteEncoder {
  AnalogEncoder encoder;

  double lastReading;
  double velocity;
  long lastTimestamp;

  public AbsoluteAnalogEncoder(int channel) {
    encoder = new AnalogEncoder(channel);

    lastTimestamp = Logger.getTimestamp();
    lastReading = getPosition();
  }

  public double getRaw() {
    return encoder.get();
  }

  public double getPosition() {
    return Radian.convertFrom(this.getRaw() * 360, Degrees);
  }

  public double getVelocity() {
    return calculateVelocity(getPosition(), Logger.getTimestamp());
  }

  public double calculateVelocity(double currentPositionRad, long currentTimestampMicros) {
    double deltaRad = MathUtil.angleModulus(currentPositionRad - lastReading);

    double deltaTimeSec = Seconds.convertFrom(currentTimestampMicros - lastTimestamp, Microseconds);

    double velocityRadPerSec = deltaRad / deltaTimeSec;

    lastReading = currentPositionRad;
    lastTimestamp = currentTimestampMicros;

    return velocityRadPerSec;
  }
}
