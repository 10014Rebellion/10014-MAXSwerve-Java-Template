package frc.robot.commands.LEDCommands;

import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.LEDInterface;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
public class commandLEDNotePickup extends Command{
    LEDInterface LED;

    public commandLEDNotePickup(LEDInterface LED) {
        this.LED = LED;
        addRequirements(LED);
    }

    @Override 
    public void initialize() {
        if (IndexerConstants.robotHasNote) {
            if (DriverStation.getAlliance().equals("RED")) {
                LED.setTransitionHueToHueValues(0, 10);
            }
            else {
                LED.setTransitionHueToHueValues(100, 10);
            }
        }
        else {
            if (DriverStation.getAlliance().equals("RED")) {
                LED.setTransitionHueToHueValues(0, 0);
            }
            else {
                LED.setTransitionHueToHueValues(100, 100);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {   
        if (IndexerConstants.robotHasNote) {
            if (DriverStation.getAlliance().equals("RED")) {
                LED.setTransitionHueToHueValues(0, 10);
            }
            else {
                LED.setTransitionHueToHueValues(100, 10);
            }
        }
        else {
            if (DriverStation.getAlliance().equals("RED")) {
                LED.setTransitionHueToHueValues(0, 0);
            }
            else {
                LED.setTransitionHueToHueValues(100, 100);
            }
        }
        
    }

    @Override
    public boolean isFinished() {
        return IndexerConstants.robotHasNote;
    }
}
