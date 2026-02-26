package frc.robot.commands.swervedrive;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Landmarks;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class AimAndDriveCommand extends Command {

    private static final Angle kAimTolerance = Degrees.of(5);

    private final SwerveSubsystem swerve;
    private final SwerveInputStream stream;

    public AimAndDriveCommand(
        SwerveSubsystem swerve,
        DoubleSupplier forwardInput,
        DoubleSupplier leftInput
    ) {
        this.swerve = swerve;

        // Tworzymy strumień YAGSL - zastępuje DriveInputSmoother i ręczną matematykę
        this.stream = SwerveInputStream.of(
            swerve.getSwerveDrive(), 
            forwardInput, 
            leftInput
        )
        // Podpinamy natywny kontroler kąta YAGSL (czyta wartości P/I/D z plików konfiguracyjnych JSON)
        .withControllerRotationAxis(() -> {
            Rotation2d targetHeading = getDirectionToHub();
            return swerve.getSwerveDrive().getSwerveController().headingCalculate(
                swerve.getPose().getRotation().getRadians(),
                targetHeading.getRadians()
            );
        })
        .deadband(0.1) // Automatyczna martwa strefa na drążkach
        .scaleTranslation(1.0) // Ewentualny mnożnik prędkości maksymalnej dla tej komendy
        .allianceRelativeControl(true); // YAGSL automatycznie dopasuje sterowanie przód/tył do tego, po jakiej stronie boiska zaczynasz

        addRequirements(swerve);
    }

    public AimAndDriveCommand(SwerveSubsystem swerve) {
        this(swerve, () -> 0.0, () -> 0.0);
    }

    private Rotation2d getDirectionToHub() {
        // Obliczamy pożądany kąt za pomocą zwykłej różnicy wektorów
        final Translation2d hubPosition = Landmarks.hubPosition();
        final Translation2d robotPosition = swerve.getPose().getTranslation();
        return hubPosition.minus(robotPosition).getAngle();
    }

    public boolean isAimed() {
        Rotation2d targetHeading = getDirectionToHub();
        Rotation2d currentHeading = swerve.getPose().getRotation();
        
        // Zwracamy prawdę, jeśli robot patrzy na Huba z dokładnością do naszego marginesu błędu
        double errorDegrees = Math.abs(currentHeading.minus(targetHeading).getDegrees());
        return errorDegrees < kAimTolerance.in(Degrees);
    }

    @Override
    public void execute() {
        // Cała matematyka, wygładzanie, orientacja do pola i celowanie są połykane naraz przez strumień!
        swerve.getSwerveDrive().driveFieldOriented(stream.get());
    }

    @Override
    public void end(boolean interrupted) {
        // Bezpieczne zatrzymanie układu napędowego po zakończeniu komendy (puszczeniu drążków/przycisku)
        swerve.getSwerveDrive().drive(new Translation2d(0, 0), 0, true, false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}