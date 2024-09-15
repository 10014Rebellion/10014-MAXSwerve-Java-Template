package frc.robot.subsystems;

import java.sql.Driver;
import java.time.Instant;

import org.w3c.dom.css.RGBColor;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.HSVLEDColor;
import frc.robot.Constants.RGBLEDColor;
import frc.robot.Constants.ShooterConstants;

public class LEDInterface extends SubsystemBase{
    
    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;

    private int initialHue;
    private int finalHue;
    private int[] currentRGB = {0,0,0};
    private int[] finalRGB = {0,0,0};
    private int currentHue;
    private int transitionSpeed;
    private int hueVariation = 180;
    private int startingLED;

    private boolean instantTransition;

    public LEDInterface() {
        led = new AddressableLED(0);
        ledBuffer = new AddressableLEDBuffer(18); // Update this with the correct lenth later

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
        this.startingLED = 0;
        this.initialHue = currentHue;
        this.finalHue = finalHue % hueVariation;
        this.transitionSpeed = transitionSpeed;
        this.instantTransition = false;
    }

    public void setTransitionRGBValues (int[] color, int transitionSpeed) {
        this.startingLED = 0;
        this.finalRGB = color;
        this.transitionSpeed = transitionSpeed;
        this.instantTransition = true;
    }

    public void setInstantHueTransitionValues(int finalHue) {
        this.finalHue = finalHue;
        this.instantTransition = true;
    }

    public void transitionHueToHue() {
        if (instantTransition == false) {
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
        else {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                ledBuffer.setHSV(i, finalHue, 255, 128);
            }
        }
        
    }
    public void transitionRGB() {
        if (instantTransition == false) {
            startingLED += transitionSpeed;
            if (startingLED >= ledBuffer.getLength()) {
                startingLED = ledBuffer.getLength();
            }
            
            //(initialHue - finalHue) / (ledBuffer.getLength() - startingLED);

            for (int i = 0; i < ledBuffer.getLength(); i++) {
                // NOTE: For some reason the chinese manufacturers decided
                // That RGB was no good and so the order is GRB,
                // Hence the weird list addresses
                if (i >= (ledBuffer.getLength() - startingLED) - 1) {
                    ledBuffer.setRGB(i, finalRGB[1], finalRGB[0], finalRGB[2]);
                }
                else {
                    if (i >= 1) {
                        ledBuffer.setRGB(i - 1, currentRGB[1], currentRGB[0], currentRGB[2]);
                    }
                    currentRGB = finalRGB; 
                }
                //System.out.println(i);
            }
        }
        else {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                ledBuffer.setRGB(i, finalRGB[1], finalRGB[0], finalRGB[2]);
            }
        }
        
    }

    public void rainbowUnicornVomit() {
        startingLED += 1;
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            int currentID = (startingLED + i) % ledBuffer.getLength();
            ledBuffer.setHSV(currentID, i*(180/ledBuffer.getLength()), 255, 128);
            //System.out.println(i*(180/ledBuffer.getLength()));
        }
    }

    public void resetStartingLED() {
        startingLED = 0;
    }
    
    public Command colorToBlueTransition() {
        return new InstantCommand(() -> setTransitionRGBValues(RGBLEDColor.blue, transitionSpeed));
    }
    public Command colorToRedTransition() {
        return new InstantCommand(() -> setTransitionRGBValues(RGBLEDColor.red, transitionSpeed));
    }

    public Command colorToOrangeTransition() {
        return new InstantCommand(() -> setTransitionRGBValues(RGBLEDColor.orange, transitionSpeed));
    }

    public Command colorToGreenTransition() {
        return new InstantCommand(() -> setTransitionRGBValues(RGBLEDColor.green, transitionSpeed));
    }

    public Command readyToShoot(){
        return new InstantCommand(() -> {
            setTransitionRGBValues(RGBLEDColor.green, 1);
            updateLEDandTelem();
        });
    }

    public Command hasNote() {
        return new InstantCommand(() -> {
            setTransitionRGBValues(RGBLEDColor.orange, 1);
            updateLEDandTelem();
        });
    }

    public Command defaultLED(){
        return new InstantCommand(() -> {
            if (DriverStation.getAlliance().toString().equals("Red")) 
                setTransitionRGBValues(RGBLEDColor.red, 1);
            else 
                setTransitionRGBValues(RGBLEDColor.blue, 1);

            updateLEDandTelem();
        });
    }
    
    public void updateLEDandTelem(){
        transitionRGB();
        led.setData(ledBuffer);
        SmartDashboard.putBooleanArray("Aimed, Flywheels, Arm", 
        new boolean[] {DriveConstants.aimedAtTarget, FlywheelConstants.flywheelsAtSetpoint, ShooterConstants.armAtSetpoint});
    }

    @Override
    public void periodic() {
        // if (DriveConstants.aimedAtTarget && FlywheelConstants.flywheelsAtSetpoint
        // && ShooterConstants.armAtSetpoint) {
        //     setTransitionRGBValues(RGBLEDColor.green, 1);
        // }
        // else if (IndexerConstants.robotHasNote) {
        //     setTransitionRGBValues(RGBLEDColor.orange, 1);
        // }
        // else {
        //     if (DriverStation.getAlliance().toString().equals("Red")) {
        //         setTransitionRGBValues(RGBLEDColor.red, 1);
        //     }
        //     else {
        //         setTransitionRGBValues(RGBLEDColor.blue, 1);
        //     }
        // }
        // transitionRGB();
        //rainbowUnicornVomit();
        // led.setData(ledBuffer);
        /*SmartDashboard.putNumber("Current R Color", finalRGB[0]);
        SmartDashboard.putNumber("Current G Color", finalRGB[1]);
        SmartDashboard.putNumber("Current B Color", finalRGB[2]);*/
        // SmartDashboard.putBooleanArray("Aimed, Flywheels, Arm", 
        // new boolean[] {DriveConstants.aimedAtTarget, FlywheelConstants.flywheelsAtSetpoint, ShooterConstants.armAtSetpoint});
        //System.out.println("GRELEKJLDKSJ");
    }
}
