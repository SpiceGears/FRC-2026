package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeRollersSubsystem extends SubsystemBase {

    public enum RollerSpeed {
        STOP(0.0),
        INTAKE(1.0),
        OUTTAKE(-0.8);

        public final double percentOutput;
        private RollerSpeed(double percentOutput) { this.percentOutput = percentOutput; }
    }

    private final PWMSparkMax intakeMaster;

    public IntakeRollersSubsystem() {
        intakeMaster = new PWMSparkMax(0);
        intakeMaster.setInverted(false);
    }

    public void setRollers(RollerSpeed speed) {
        intakeMaster.set(speed.percentOutput * -1.0); 
    }

    public Command runRollersCommand(RollerSpeed speed) {
        return Commands.startEnd(
            () -> setRollers(speed), 
            () -> setRollers(RollerSpeed.STOP), 
            this
        ).withName("Rollers.Run");
    }
}