package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    
    public enum LedColor {
        RED(0),
        ORANGE(1),
        YELLOW(2),
        GREEN(3), 
        CYAN(4),
        BLUE(5),
        MAGENTA(6),
        PINK(7);

        public final int slot;
        LedColor(int slot) {
            this.slot = slot;
        }
    }

    private final PWM controller = new PWM(3);

    private final SendableChooser<LedColor> colorChooser = new SendableChooser<>();

    public LEDSubsystem() {
        controller.setBoundsMicroseconds(2000, 1501, 1500, 1499, 1000);

        colorChooser.setDefaultOption("Idle", null);
        for(LedColor color : LedColor.values()) {
          colorChooser.addOption(color.name(), color);
        }

        SmartDashboard.putData("LED Pattern", colorChooser);
    }

    public void setColor(LedColor color) {
        // Mapujemy slot (0-15) na zakres -1.0 do 1.0
        if (color == null) {
                setIdle();
                return;
        }
        // Wzór: (slot / 7.5) - 1.0
        int arduinoSlot = color.slot * 2;
        double speed = (arduinoSlot / 7.5) - 1.0;
        controller.setSpeed(speed);
    }

    public void setIdle() {
        controller.setDisabled();
    }

    public Command setColorCommand(LedColor color) {
      return this.run(() -> this.setColor(color));
    }

    public Command getDefaultDashboardCommand() {
        return this.run(() -> setColor(colorChooser.getSelected()))
                   .withName("LedDashboardDefault");
    }
}