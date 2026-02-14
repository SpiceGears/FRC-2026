package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.spark.SparkBase.Faults;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs 
    {
        public Rotation2d intakePosition = new Rotation2d();
        public double intakeSpeed = 0.0;


        public double spinnerSpeed =  0.0;

        // intake motors volts and ampers
        public double spinnerVoltage = 0.0;
        public double spinnerAmperage = 0.0;
        public double intakeVoltage = 0.0;
        public double intakeAmperage = 0.0;

        //faults
        public Faults spinnerFaults = new Faults(0);
        public Faults extenderFaults = new Faults(0);


    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void setSpinnerSpeed(double speed) {}

    public default void setIntakePosition(double setpoint) {}

    public default void extendIntake() { this.setIntakePosition(IntakeConstants.extendedPosition);}

    public default void foldIntake() { this.setIntakePosition(IntakeConstants.foldedPosition);}
}
