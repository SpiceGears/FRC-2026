package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortMap;

public class PasserSubsystem extends SubsystemBase {

    private final PWMSparkMax passerMotor;

    public PasserSubsystem() {
        passerMotor = new PWMSparkMax(PortMap.SHOOTER_PASSER_MOTOR_PWM);
        
        passerMotor.setInverted(false); 
    }

    public Command runPasserCommand(double speed) {
        return this.runEnd(
            () -> passerMotor.set(speed),
            () -> passerMotor.set(0)
        ).withName("Passer.Run");
    }
}