package frc.robot.subsystems.intake;

public class IntakeConstants {

  public static final int canSpinnerMotorId = 20;
  public static final int canIntakeExtenderMotorId = 21;

  public static final double extendedPosition = 1000;
  public static final double foldedPosition = 0;

  public static final double motorReduction = 4 * 4 * 5;
  public static final double beltReduction = 2.0;

  public static final double extenderClosedLoopControllerP = 0.5;
  public static final double extenderClosedLoopControllerI = 0.0;
  public static final double extenderClosedLoopControllerD = 0.0;
  public static final double extenderClosedLoopControllerMinimumOutput = -1.0;
  public static final double extenderClosedLoopControllerMaximumOutput = 1.0;
}
