package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDInterface extends SubsystemBase{
    
    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;

    private int initialHue;
    private int finalHue;
    private int currentHue;
    private int transitionSpeed;
    private int hueVariation = 180;
    private int startingLED;

    public LEDInterface() {
        led = new AddressableLED(0);
        ledBuffer = new AddressableLEDBuffer(36); // Update this with the correct lenth later

        led.setLength(ledBuffer.getLength());

        led.setData(ledBuffer);
        led.start();

        startingLED = 0;
    }

    public void setStripColor(int red, int green, int blue) {

        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, red, green, blue);
        }

        led.setData(ledBuffer);
    }

    public void setTransitionHueToHueValues(int finalHue, int transitionSpeed) {
        startingLED = 0;
        this.initialHue = currentHue;
        this.finalHue = finalHue % hueVariation;
        this.transitionSpeed = transitionSpeed;
    }

    public void transitionHueToHue() {
        if (startingLED >= 71) {
            startingLED = 71;
        }
        int increment = 1;
        if (initialHue > finalHue) {
            increment = -1;
        }
        else {
            increment = 1;
        }
        startingLED += transitionSpeed;
        //(initialHue - finalHue) / (ledBuffer.getLength() - startingLED);

        for (int i = 0; i < ledBuffer.getLength(); i++) {
            
            if (i >= (ledBuffer.getLength() - startingLED) - 1) {
             ledBuffer.setHSV(i, finalHue, 255, 128);
            }
            else {
                if (i >= 1) {
                    ledBuffer.setHSV(i - 1,initialHue, 255, 128);
                }
                
                ledBuffer.setHSV(i, -Math.abs((finalHue - initialHue) / 2), 255, 128);
                currentHue = finalHue;
            }

            /*else {
                ledBuffer.setHSV(i, ((increment * (i + startingLED))), 255, 128);
            }*/
        }
    }

    public void resetStartingLED() {
        startingLED = 0;
    }
    
    public Command colorToBlueTransition() {
        return new InstantCommand(() -> setTransitionHueToHueValues(108, 1));
    }
    public Command colorToRedTransition() {
        return new InstantCommand(() -> setTransitionHueToHueValues(0, 1));
    }

    public Command colorToOrangeTransition() {
        return new InstantCommand(() -> setTransitionHueToHueValues(10, 1));
    }

    public Command colorToPurpleTransition() {
        return new InstantCommand(() -> setTransitionHueToHueValues(170, 1));
    }

    @Override
    public void periodic() {
        //startingLED += 1;
        transitionHueToHue();
        led.setData(ledBuffer);
    }
}
