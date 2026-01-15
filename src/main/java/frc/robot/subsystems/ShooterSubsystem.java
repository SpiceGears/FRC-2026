package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    public Talon shooterMaster;
    public Talon shooterSlave;

    public ShooterSubsystem() {
        shooterMaster = new Talon(0);
        shooterSlave = new Talon(1);

        shooterMaster.setInverted(false);
        shooterSlave.setInverted(true);
    }

    public void setShooterPower(double power) {
        shooterMaster.set(power);
        shooterSlave.set(power);
    }

    public void stopShooter() {
        shooterMaster.stopMotor();
        shooterSlave.stopMotor();
    }

    public double getIntakePower() {
        return shooterMaster.get();
    }

    public void setShooterVolts(double volts) {
        shooterMaster.setVoltage(volts);
        shooterSlave.setVoltage(volts);
    }
}
