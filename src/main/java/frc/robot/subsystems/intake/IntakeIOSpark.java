package frc.robot.subsystems.intake;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;

public class IntakeIOSpark implements IntakeIO {

  private SparkMax extenderMotor =
      new SparkMax(IntakeConstants.canIntakeExtenderMotorId, MotorType.kBrushless);
  private SparkMax spinnerMotor =
      new SparkMax(IntakeConstants.canSpinnerMotorId, MotorType.kBrushless);

  private AbsoluteEncoder extenderEncoder;
  SparkClosedLoopController extenderController;

  public IntakeIOSpark() {
    SparkMaxConfig extenderConfig = new SparkMaxConfig();

    extenderConfig
        .openLoopRampRate(0.5)
        .closedLoopRampRate(0.5)
        .smartCurrentLimit(30)
        .idleMode(IdleMode.kBrake);

    extenderConfig
        .closedLoop
        .p(IntakeConstants.extenderClosedLoopControllerP)
        .i(IntakeConstants.extenderClosedLoopControllerI)
        .d(IntakeConstants.extenderClosedLoopControllerD)
        .outputRange(
            IntakeConstants.extenderClosedLoopControllerMinimumOutput,
            IntakeConstants.extenderClosedLoopControllerMaximumOutput)
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    extenderMotor.configure(
        extenderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    extenderController = extenderMotor.getClosedLoopController();
    extenderEncoder = extenderMotor.getAbsoluteEncoder();

    SparkMaxConfig spinnerConfig = new SparkMaxConfig();
    spinnerConfig.inverted(true).idleMode(IdleMode.kCoast).smartCurrentLimit(30);

    spinnerMotor.configure(
        spinnerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeAmperage = extenderMotor.getOutputCurrent();
    inputs.intakeVoltage = extenderMotor.getAppliedOutput() * extenderMotor.getBusVoltage();
    inputs.intakePosition = new Rotation2d(extenderEncoder.getPosition());
    inputs.intakeSpeed = extenderEncoder.getVelocity();

    inputs.spinnerAmperage = spinnerMotor.getOutputCurrent();
    inputs.spinnerVoltage = spinnerMotor.getAppliedOutput() * spinnerMotor.getBusVoltage();
    inputs.spinnerSpeed = spinnerMotor.getEncoder().getVelocity();

    inputs.spinnerFaults = spinnerMotor.getFaults();
    inputs.extenderFaults = extenderMotor.getFaults();
  }

  @Override
  public void setSpinnerSpeed(double speed) {
    spinnerMotor.set(speed);
  }

  @Override
  public void setIntakePosition(double setpoint) {
    extenderController.setSetpoint(setpoint, ControlType.kPosition);
  }
}
