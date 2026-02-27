package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.led.LEDSubsystem.LedColor;

public class ShooterSubsystem extends SubsystemBase {

    public final FlywheelSubsystem flywheel;
    public final PasserSubsystem passer;
    public final HoodSubsystem hood;

    private final DoubleSupplier distanceToHubSupplier;

    private final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();

    public ShooterSubsystem(FlywheelSubsystem flywheel, PasserSubsystem passer, 
      HoodSubsystem hood, DoubleSupplier distanceToHubSupplier) {

        this.flywheel = flywheel;
        this.passer = passer;
        this.hood = hood;
        this.distanceToHubSupplier = distanceToHubSupplier;

        loadShooterMaps();

        SmartDashboard.putNumber("TestShooter/Target RPM", 3000.0);
        SmartDashboard.putNumber("TestShooter/Target Hood (mm)", 10.0);
    }

    private void loadShooterMaps() {
        rpmMap.put(1.24,3500.0);
        rpmMap.put(1.5, 3600.0);
        rpmMap.put(1.71, 3700.0);
        rpmMap.put(2.11, 3900.0);
        rpmMap.put(2.56, 4000.0);
        rpmMap.put(2.57, 4300.0);
        rpmMap.put(2.73, 4500.0);
        rpmMap.put(3.14, 4150.0);
        rpmMap.put(3.28, 4350.0);
        rpmMap.put(3.5, 4300.0);
        rpmMap.put(3.78, 4400.0);
        rpmMap.put(4.0, 4700.0);
        rpmMap.put(4.2, 4650.0);
        rpmMap.put(4.56, 4800.0);
        rpmMap.put(4.6, 4800.0);
        rpmMap.put(5.2, 5000.0);

        hoodMap.put(1.24,10.0);
        hoodMap.put(1.5, 11.0);
        hoodMap.put(1.71, 12.0);
        hoodMap.put(2.11, 14.0);
        hoodMap.put(2.56, 14.0);
        hoodMap.put(2.57, 15.0);
        hoodMap.put(2.73, 16.0);
        hoodMap.put(3.14, 18.0);
        hoodMap.put(3.28, 19.0);
        hoodMap.put(3.5, 16.5);
        hoodMap.put(3.78, 18.00);
        hoodMap.put(4.0, 19.0);
        hoodMap.put(4.2, 18.0);
        hoodMap.put(4.56, 20.0);
        hoodMap.put(4.6, 20.0);
        hoodMap.put(5.2, 22.5);
    }

    @Override
    public void periodic() {
        // --- TELEMETRIA DIAGNOSTYCZNA NA ŻYWO ---

        SmartDashboard.putNumber("TestShooter/DISTANCE TO HUB (m)", distanceToHubSupplier.getAsDouble());
        
        // Stan Flywheela
        SmartDashboard.putNumber("TestShooter/Current RPM", flywheel.getVelocity().in(RPM));
        SmartDashboard.putBoolean("TestShooter/Is Flywheel Ready", flywheel.isAtTargetVelocity());

        // Stan Kaptura
        SmartDashboard.putNumber("TestShooter/Current Hood (mm)", hood.getTargetExtensionMm());
        
        // Ogólna gotowość (żebyś widział zieloną/czerwoną lampkę na dashboardzie)
        boolean isReadyToShoot = flywheel.isAtTargetVelocity();
        SmartDashboard.putBoolean("TestShooter/READY TO FIRE", isReadyToShoot);
    }

    public Command shootCommand(FeederSubsystem feeder, LEDSubsystem leds) {
        return Commands.sequence(
            
            // KROK 1: Pobierz dystans na żywo, wyciągnij z map wartości i przypisz do silników
            Commands.runOnce(() -> {
                double distance = distanceToHubSupplier.getAsDouble();
                
                double targetRPM = rpmMap.get(distance);
                double targetHoodMm = hoodMap.get(distance);

                flywheel.setTargetVelocity(RPM.of(targetRPM));
                flywheel.spinUpToVelocity(RPM.of(targetRPM));
                
                hood.setExtensionMm(targetHoodMm);
            }, this),

            // KROK 2: Poczekaj na rozpędzenie Flywheela i wysunięcie Kaptura
            Commands.waitUntil(() -> flywheel.isAtTargetVelocity()),

            // KROK 3: Odpal systemy podające piłkę i LEDy
            Commands.parallel(
                leds.setColorCommand(LedColor.MAGENTA),
                passer.runPasserCommand(0.9),
                feeder.feedShooterCommand(1.0)
            )
            
        ).finallyDo(() -> {
            // KROK 4: Bezpieczne uśpienie całego systemu po puszczeniu przycisku
            flywheel.stopControl();
            leds.idle();
        })
        .withName("Shooter.SmartShootSequence");
    }

    public Command prepareShoot(FeederSubsystem feeder, LEDSubsystem leds) 
    {
        return Commands.sequence(
            Commands.runOnce(() -> {
                double distance = distanceToHubSupplier.getAsDouble();
                
                double targetRPM = rpmMap.get(distance);
                double targetHoodMm = hoodMap.get(distance);

                flywheel.setTargetVelocity(RPM.of(targetRPM));
                flywheel.spinUpToVelocity(RPM.of(targetRPM));
                
                hood.setExtensionMm(targetHoodMm);
            }, this)
        ).finallyDo( () -> {
            flywheel.stopControl();
            leds.idle();
        }
        ).withName("Shooter.SmartShootPrepare");
    }

    public Command testShootCommand(FeederSubsystem feeder, LEDSubsystem leds) {
        return Commands.sequence(
            
            // KROK 1: Odczyt wartości "z palca" i wprawienie mechanizmów w ruch
            Commands.runOnce(() -> {
                double targetRPM = SmartDashboard.getNumber("TestShooter/Target RPM", 3000.0);
                double targetHoodMm = SmartDashboard.getNumber("TestShooter/Target Hood (mm)", 10.0);

                flywheel.setTargetVelocity(RPM.of(targetRPM));
                flywheel.spinUpToVelocity(RPM.of(targetRPM));
                
                hood.setExtensionMm(targetHoodMm);
            }, this),

            // KROK 2: Czekamy na zielone światło od Flywheela i Kaptura
            Commands.waitUntil(() -> flywheel.isAtTargetVelocity()),

            // KROK 3: Odpalamy podawanie piłki i sygnalizację LED
            Commands.parallel(
                leds.setColorCommand(LedColor.MAGENTA),
                passer.runPasserCommand(0.9),
                feeder.feedShooterCommand(1.0)
            )
            
        ).finallyDo(() -> {
            // KROK 4: Bezpieczne uśpienie całego systemu po puszczeniu przycisku
            flywheel.stopControl();
            leds.idle();
            
            // Passer i Feeder mają wbudowane zatrzymanie w runEnd(), więc wyłączą się same
        })
        .withName("Shooter.TestShootSequence");
    }

    public Command shoot(FeederSubsystem feeder, LEDSubsystem leds, int set) {
      return Commands.none();
    }

    // ==========================================
    // METODA POMOCNICZA: Awaryjne zatrzymanie
    // ==========================================
    public Command stopEverythingCommand() {
        return Commands.runOnce(() -> {
            flywheel.stopControl();
            passer.runPasserCommand(0).cancel();
        }, this).withName("Shooter.EmergencyStop");
    }
}