package frc.robot.subsystems.intake;

public class IntakeConstants {


    public static final int SPINNER_CAN_ID = 20;
    public static final int INTAKE_EXTENDER_CAN_ID = 21;

    public static final double extendedPosition = 1000;
    public static final double foldedPosition = 0;

    public static final double MotorReduction = 4*4*5;
    public static final double BeltReduction = 2.0;

    public static final double extenderPID_P = 0.5;
    public static final double extenderPID_I = 0.0;
    public static final double extenderPID_D = 0.0;
    public static final double extenderPID_MINIMUM = -1.0;
    public static final double extenderPID_MAXIMUM = 1.0;

}
