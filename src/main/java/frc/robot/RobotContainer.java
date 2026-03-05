// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.intake.IntakeCommands;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakePivotSubsystem;
import frc.robot.subsystems.IntakeRollersSubsystem;
import frc.robot.subsystems.IntakeRollersSubsystem.RollerSpeed;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.led.LEDSubsystem.LedColor;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.PasserSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
//import frc.robot.subsystems.vision.ShooterVisionAid;
//import frc.robot.subsystems.vision.limelight.AprilTagVisionSubsystem;

import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandXboxController driverXbox = new CommandXboxController(0);
  //final CommandXboxController copilotXbox = new CommandXboxController(1);

  //final AprilTagVisionSubsystem aprilTagVisionSubsystem = new AprilTagVisionSubsystem();


  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  // Establish a Sendable Chooser that will be able to be sent to the SmartDashboard, allowing selection of desired auto
  private final SendableChooser<Command> autoChooser;

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -driverXbox.getLeftY(),
                                                                        () -> -driverXbox.getLeftX())
                                                                    .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true)
                                                                               .translationHeadingOffset(true)
                                                                               .translationHeadingOffset(Rotation2d.fromDegrees(
                                                                                   0));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */


  final IntakePivotSubsystem intakePivot = new IntakePivotSubsystem();
  final IntakeRollersSubsystem intakeRollers = new IntakeRollersSubsystem();
  final FlywheelSubsystem shooterFlywheel = new FlywheelSubsystem();
  final FeederSubsystem feeder = new FeederSubsystem();
  //final ClimbSubsystem climb = new ClimbSubsystem();
  final PasserSubsystem shooterPasser = new PasserSubsystem();
  final HoodSubsystem hood = new HoodSubsystem();

  final LEDSubsystem leds = new LEDSubsystem();

  //final ShooterVisionAid svas = new ShooterVisionAid(drivebase::getPose);

  final ShooterSubsystem shooter = new ShooterSubsystem(shooterFlywheel, shooterPasser, hood, 
    () -> drivebase.getPose().getTranslation().getDistance(Landmarks.hubPosition()) );
  
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();

    
    DriverStation.silenceJoystickConnectionWarning(true);
    
    //Create the NamedCommands that will be used in PathPlanner 

    NamedCommands.registerCommand("prepareShoot", shooter.prepareShoot(feeder).withTimeout(2));
    NamedCommands.registerCommand("smartShoot", shooter.shootCommand(feeder, leds));
    NamedCommands.registerCommand("aimAtHub", drivebase.autoAimAtHubCommand());
    NamedCommands.registerCommand("aimAndShoot", Commands.sequence(drivebase.autoAimAtHubCommand(), shooter.shootCommand(feeder, leds)).withTimeout(5));
    NamedCommands.registerCommand("intakeDeploy", intakePivot.deployCommand());
    NamedCommands.registerCommand("intakeAgitate", IntakeCommands.agitate(intakePivot, intakeRollers));
    NamedCommands.registerCommand("shootWithAgitate", Commands.parallel(shooter.shootCommand(feeder, leds), IntakeCommands.agitate(intakePivot, intakeRollers)).withTimeout(8));
    NamedCommands.registerCommand("intakeBalls", intakeRollers.runRollersCommand(RollerSpeed.INTAKE).withTimeout(5));
    //NamedCommands.registerCommand("openHopper", climb.open().withTimeout(0.5));

    //Have the autoChooser pull in all PathPlanner autos as options
    autoChooser = AutoBuilder.buildAutoChooser();

    //Set the default auto (do nothing) 
    autoChooser.setDefaultOption("Do Nothing", Commands.none());
    
    //Put the autoChooser on the SmartDashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);




    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
      //drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
    }

    if (Robot.isSimulation())
    {
      Pose2d target = new Pose2d(new Translation2d(1, 4),
                                 Rotation2d.fromDegrees(90));
      //drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
      driveDirectAngleKeyboard.driveToPose(() -> target,
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(5, 2)),
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(Units.degreesToRadians(360),
                                                                                     Units.degreesToRadians(180))
                                           ));
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
      driverXbox.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
                                                     () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));

//      driverXbox.b().whileTrue(
//          drivebase.driveToPose(
//              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
//                              );

    }
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    } else
    {

      //intake.setDefaultCommand(intake.setAngleCmd(Degrees.of(90)));
      //shooter.setDefaultCommand(new ShooterCycleCommand(shooter));
      //climb.setDefaultCommand(climb.setHeight(Meters.of(0.12)));

      //hood.setDefaultCommand(new AutoHoodAdjustment(hood));
      driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyroWithAlliance)));
      //driverXbox.x().onTrue(shooter.toggleEnabledCmd());
      //driverXbox.x().whileTrue(shooter.shootAdaptable(feeder, leds));
      //driverXbox.y().whileTrue(shooter.shoot(feeder, leds, 4000));
      //driverXbox.b().whileTrue(new SwerveAimAt(drivebase));
      //driverXbox.y().onTrue(shooter.stop());
      //driverXbox.start().onTrue(intake.setAngle(Degrees.of(90)));
      driverXbox.back().whileTrue(Commands.none());
      //driverXbox.leftBumper().whileTrue(Commands.runOnce(() -> hood.extendFull()));
      //driverXbox.rightBumper().onTrue(Commands.none());
      //feeder.setDefaultCommand(new FeederFeedShooterCommand(feeder, driverXbox.rightBumper()));


      // driverXbox.povUp().whileTrue(climb.runManualCommand(-10));
      // driverXbox.povDown().whileTrue(climb.runManualCommand(10));

      
      //driverXbox.rightTrigger(0.1).whileTrue(new IntakeFuel(intake));


      /// # DRIVER SUGGESTION
      driverXbox.rightTrigger(0.1).whileTrue(
        Commands.parallel(
          drivebase.aimAtHub(
          () -> -driverXbox.getLeftY(), 
          () -> -driverXbox.getLeftX()
          ),
          shooter.shootCommand(feeder, leds)
        )
        );
      driverXbox.leftTrigger().whileTrue(shooter.shootCommand(feeder, leds));

      // driverXbox.rightTrigger(0.1).whileTrue(
      //   shooter.shootCommand(feeder, leds)
      // );
      
      //driverXbox.leftBumper().whileTrue(shooter.passFuelToShooter());
      //driverXbox.leftTrigger(0.1).whileTrue(shooterPasser.runPasserCommand(1));
      // driverXbox.povUp().onTrue(intake.adjustIntake(Degrees.of(5)));
      // driverXbox.povDown().onTrue(intake.adjustIntake(Degrees.of(-5)));

      //driverXbox.povUp().onTrue(Commands.runOnce(() -> hood.setExtensionMm(28)));
      //driverXbox.povDown().onTrue(Commands.runOnce(() -> hood.setExtensionMm(5)));
      //driverXbox.povRight().onTrue(intakePivot.homingCommand());

      driverXbox.povLeft().whileTrue(intakePivot.deployCommand());
      
      // driverXbox.leftBumper().whileTrue(
      //   IntakeCommands.agitate(intakePivot, intakeRollers)
      // );

      driverXbox.leftBumper().whileTrue(
        intakeRollers.runRollersCommand(RollerSpeed.OUTTAKE)
        .alongWith(feeder.feedShooterCommand(-1))
        .alongWith(shooter.reversePasser())
        .alongWith(leds.holdColorCommand(LedColor.RED))
        .finallyDo(
          () -> leds.idle()
        )
        );
      driverXbox.rightBumper().whileTrue(intakeRollers.runRollersCommand(RollerSpeed.INTAKE).alongWith(intakePivot.deployCommand()));
      //driverXbox.povDown().whileTrue(IntakeCommands.agitate(intakePivot, intakeRollers));

      leds.setDefaultCommand(leds.getDefaultDashboardCommand());

      // copilotXbox.rightBumper().whileTrue(
      //   intakeRollers.runRollersCommand(RollerSpeed.OUTTAKE)
      //   .alongWith(feeder.feedShooterCommand(-1))
      //   .alongWith(shooter.reversePasser())
      //   .alongWith(leds.holdColorCommand(LedColor.RED))
      //   .finallyDo(
      //     () -> leds.idle()
      //   )
      //   );

      // copilotXbox.leftBumper().whileTrue(intakePivot.deployCommand());

      //driverXbox.leftTrigger().whileTrue(new AimAndDriveCommand(drivebase));
//       driverXbox.leftTrigger().whileTrue(
//         drivebase.aimAtHub(
//         () -> -driverXbox.getLeftY(), 
//         () -> -driverXbox.getLeftX()
//       )
// );
      //driverXbox.leftBumper().whileTrue(leds.setColorCommand(LedColor.BLUE));
    }

  }

  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // Pass in the selected auto from the SmartDashboard as our desired autnomous commmand 
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
