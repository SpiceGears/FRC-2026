package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    
    private static final int PORT = 3;
    private static final int NUM_LEDS = 32;

    private static final double BRIGHTNESS = 0.3;

    private final int COOLING = 70; 

    private final int SPARKING = 120;

    public enum LedColor {
        RED(255, 0, 0),
        ORANGE(255, 120, 0),
        YELLOW(255, 255, 0),
        GREEN(0, 255, 0), 
        CYAN(0, 255, 255),
        BLUE(0, 0, 255),
        MAGENTA(255, 0, 255),
        PINK(255, 100, 150);

        public final int r, g, b;
        LedColor(int r, int g, int b) {
            this.r = r; 
            this.g = g; 
            this.b = b;
        }
    }

    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_ledBuffer;
    private final SendableChooser<LedColor> m_colorChooser = new SendableChooser<>();
    
    // Zmienna do animacji tęczy
    private int m_rainbowFirstPixelHue = 0;
    
    // Tablica przechowująca "temperaturę" dla efektu ognia
    private final int[] heat = new int[NUM_LEDS];

    public LEDSubsystem() {
        m_led = new AddressableLED(PORT);
        m_ledBuffer = new AddressableLEDBuffer(NUM_LEDS);
        m_led.setLength(m_ledBuffer.getLength());
        
        m_led.setData(m_ledBuffer);
        m_led.start();

        m_colorChooser.setDefaultOption("Rainbow (Idle)", null);
        for(LedColor color : LedColor.values()) {
          m_colorChooser.addOption(color.name(), color);
        }

        SmartDashboard.putData("LED Pattern", m_colorChooser);
    }

    /**
     * Główna metoda ustawiająca kolory lub włączająca tęczę.
     */
    public void setColor(LedColor color) {
        if (color == null) {
            animateFire();
        } else {
            // Ustawienie stałego koloru dla wszystkich diod
            for (int i = 0; i < m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setRGB(i, (int)(color.r * BRIGHTNESS), (int)(color.g * BRIGHTNESS), (int) (color.b * BRIGHTNESS));
            }
            m_led.setData(m_ledBuffer);
        }
    }

    private void runRainbow() {
        // Obliczamy wartość V (Value) dla palety HSV (od 0 do 255)
        int v = (int) (255 * BRIGHTNESS);

        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            final int hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
            // Podajemy nasze przeliczone 'v' jako jasność
            m_ledBuffer.setHSV(i, hue, 255, v); 
        }
        
        m_led.setData(m_ledBuffer);
        
        m_rainbowFirstPixelHue += 3;
        m_rainbowFirstPixelHue %= 180;
    }

    /**
     * Algorytm animacji ognia (od dołu do góry).
     */
    public void animateFire() {
    int ledLength = heat.length;

    // 1. CHŁODZENIE: Każdy piksel lekko stygnie.
    // Dzięki temu ogień naturalnie zanika im wyżej się znajduje.
    for (int i = 0; i < ledLength; i++) {
      int cooldown = (int) (Math.random() * ((COOLING * 10) / ledLength + 2));
      heat[i] = Math.max(0, heat[i] - cooldown);
    }

    // 2. KONWEKCJA: Ciepło unosi się do góry.
    // Każdy piksel "kradnie" trochę ciepła od dwóch pikseli poniżej siebie.
    for (int k = ledLength - 1; k >= 2; k--) {
      heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2]) / 3;
    }

    // 3. ISKRZENIE: Losowo dorzucamy "drewno do pieca" na samym dole taśmy.
    if (Math.random() * 255 < SPARKING) {
      int y = (int) (Math.random() * 3); // Iskry pojawiają się na 3 najniższych LED-ach
      heat[y] = Math.min(255, heat[y] + (int) (Math.random() * 95 + 160));
    }

    // 4. MAPOWANIE KOLORÓW: Przekładamy temperaturę na kolory wpilibowego bufora
    for (int j = 0; j < ledLength; j++) {
      setPixelHeatColor(j, heat[j]);
    }

    m_led.setData(m_ledBuffer);
  }

  /**
   * Pomocnicza metoda mapująca "ciepło" na piękne przejścia kolorów: 
   * Czarny -> Czerwony -> Pomarańczowy -> Żółty -> Biały
   */
  private void setPixelHeatColor(int pixel, int temperature) {
    // Skalujemy temperaturę (0-255) na 3 strefy kolorystyczne (0-191)
    int t192 = (int) Math.round((temperature / 255.0) * 191);
    int heatramp = t192 & 0x3F; // Skala 0-63
    heatramp <<= 2; // Rozciągamy do 0-252

    // Przypisanie kolorów w zależności od strefy
    if (t192 > 0x80) {                     // Najgorętsze (Żółto-Białe iskry na szczycie)
      m_ledBuffer.setRGB(pixel, 255, 255, heatramp);
    } else if (t192 > 0x40) {              // Środek (Żywy Pomarańcz)
      m_ledBuffer.setRGB(pixel, 255, heatramp, 0);
    } else {                               // Najchłodniejsze (Czerwień i czerń wygasania)
      m_ledBuffer.setRGB(pixel, heatramp, 0, 0);
    }
  }

    // --- KOMENDY ---

    public Command holdColorCommand(LedColor color) {
        return this.run(() -> this.setColor(color))
                   .withName("HoldColor_" + color.name());
    }

    /**
     * Komenda odpalająca animację ognia.
     */
    public Command fireCommand() {
        return this.run(this::animateFire)
                   .withName("FireAnimation");
    }

    /**
     * Komenda domyślna, czytająca z dashboardu.
     */
    public Command getDefaultDashboardCommand() {
        return this.run(() -> this.setColor(m_colorChooser.getSelected()))
                   .withName("LedDashboardDefault");
    }
}