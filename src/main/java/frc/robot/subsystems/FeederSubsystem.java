package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortMap;

public class FeederSubsystem extends SubsystemBase {

  private final PWMSparkMax feederMotor;

  public FeederSubsystem() {
    feederMotor = new PWMSparkMax(PortMap.FEEDER_MOTOR_PWM);
    
    feederMotor.setInverted(false); 
  }

  public Command feedShooterCommand(double speed) {
    return this.runEnd(
        () -> feederMotor.set(speed), 
        () -> feederMotor.set(0)
    ).withName("Feeder.Run");
  }

  public Command feedPulseCommand(double forwardSpeed, double reverseSpeed) {
  return Commands.sequence(

      Commands.startEnd(
          () -> feederMotor.set(forwardSpeed),
          () -> feederMotor.set(0),
          this
      ).withTimeout(2.0),

      Commands.startEnd(
          () -> feederMotor.set(reverseSpeed),
          () -> feederMotor.set(0),
          this
      ).withTimeout(1.0)

  ).withName("Feeder.Pulse");
}
}