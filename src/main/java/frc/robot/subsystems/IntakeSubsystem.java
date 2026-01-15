package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private Talon intakeMaster;

    public IntakeSubsystem() {
        intakeMaster = new Talon(2);

        intakeMaster.setInverted(false);
    }

    public void setIntakePower(double power) {
        intakeMaster.set(power);
    }

    public void stopIntake() {
        intakeMaster.stopMotor();
    }

    public double getIntakePower() {
        return intakeMaster.get();
    }

    public void setIntakeVolts(double volts) {
        intakeMaster.setVoltage(volts);
    }
}
