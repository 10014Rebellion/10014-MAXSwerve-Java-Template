package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.Constants.photonConstants;
import frc.robot.utils.TunableNumber;

public class doubleShooterFlywheels extends SubsystemBase{ 

    Flywheel leftFlywheel;
    Flywheel rightFlywheel;

    //private TunableNumber leftFlywheelVelocityTunableNumber, rightFlywheelVelocityTunableNumber;
    private TunableNumber flywheelP, flywheelV, flywheelD;
    private InterpolatingDoubleTreeMap flywheelVelocityMap;

    public doubleShooterFlywheels() {
        flywheelP = new TunableNumber("Flywheel P");
        flywheelV = new TunableNumber("Flywheel V");
        flywheelD = new TunableNumber("Flywheel D");

        flywheelD.setDefault(FlywheelConstants.kFlywheelD);
        flywheelP.setDefault( FlywheelConstants.kFlywheelP);
        flywheelV.setDefault(FlywheelConstants.kFlywheelFF);

        leftFlywheel = new Flywheel(FlywheelConstants.kLeftFlywheelMotorCanID, FlywheelConstants.kLeftFlywheelMotorCurrentLimit, false);
        leftFlywheel.initPID(flywheelP.get(), flywheelD.get(), flywheelV.get());
        rightFlywheel = new Flywheel(FlywheelConstants.kRightFlywheelMotorCanID, FlywheelConstants.kRightFlywheelMotorCurrentLimit, true);
        leftFlywheel.initPID(flywheelP.get(), flywheelD.get(), flywheelV.get());

        flywheelVelocityMap = new InterpolatingDoubleTreeMap();
        populateFlywheelVelocityMap();

        //leftFlywheelVelocityTunableNumber = new TunableNumber("Tunable Left Flywheel Velocity");
        //rightFlywheelVelocityTunableNumber = new TunableNumber("Tunable Right Flywheel Velocity");
        

        //leftFlywheelVelocityTunableNumber.setDefault(0);
        //rightFlywheelVelocityTunableNumber.setDefault(0);

        
        // Good D: 0.0001
        // Good P: 0.0003
        // Good V: 0.00017
        //System.out.println(leftFlywheelController.getSmartMotionAccelStrategy(0))
    }

    public void setLeftFlywheelVelocity(double targetVelocity) {
        leftFlywheel.setTargetVelo(targetVelocity);
    }

    public void setRightFlywheelVelocity(double targetVelocity) {
        rightFlywheel.setTargetVelo(targetVelocity);
    }

    public void setBothFlywheelVelocity(double leftTargetVelocity, double rightTargetVelocity) {
        setLeftFlywheelVelocity(leftTargetVelocity);
        setRightFlywheelVelocity(rightTargetVelocity);
    }
    
    public boolean flywheelsAtSetpoint() {
        //double offsetVelocityReference = flywheelVelocityReference * flywheelVelocityOffset;
        return (leftFlywheel.atSetpoint() && rightFlywheel.atSetpoint());
        
    }

    public double getLeftFlywheelVelocity() {
        return leftFlywheel.getVelo();
    }

    public double getRightFlywheelVelocity() {
        return rightFlywheel.getVelo();
    }

    public void tuneLeftFlywheelVelocity() {
       // setLeftFlywheelVelocity(leftFlywheelVelocityTunableNumber.get());
    }

    public void tuneRightFlywheelVelocity() {
       // setRightFlywheelVelocity(rightFlywheelVelocityTunableNumber.get());
    }

    public void setFlywheelP() {
        leftFlywheel.setP(flywheelP.get());
        rightFlywheel.setP(flywheelP.get());
    }

    private void setFlywheelV() {
        leftFlywheel.setFF(flywheelV.get());
        rightFlywheel.setFF(flywheelV.get());
    }

    private void setFlywheelD() {
        leftFlywheel.setD(flywheelD.get());
        rightFlywheel.setD(flywheelD.get());
    }
    
    public void periodic() {

        SmartDashboard.putNumber("Current Left Flywheel Velocity", getLeftFlywheelVelocity());
        SmartDashboard.putNumber("Current Right Flywheel Velocity", getRightFlywheelVelocity());
        SmartDashboard.putNumber("Left Flywheel Current", leftFlywheel.getCurrent());
        SmartDashboard.putNumber("Right Flywheel Current", rightFlywheel.getCurrent());

        /*if (leftFlywheelVelocityTunableNumber.hasChanged()) {
            tuneLeftFlywheelVelocity();
        }

        if(rightFlywheelVelocityTunableNumber.hasChanged()) {
            tuneRightFlywheelVelocity();
        }*/
    
        if (flywheelP.hasChanged()) setFlywheelP();
    
        if (flywheelV.hasChanged()) setFlywheelV();
    
        if (flywheelD.hasChanged()) setFlywheelD();

        SmartDashboard.putNumber("Left Flywheel set value", leftFlywheel.getAppliedOutput());
    }
    
    // Manual control.
    public void setFlywheelVoltage(double targetVoltage) {
        setFlywheelVoltage(targetVoltage, targetVoltage);
    }

    public void setFlywheelVoltage(double leftTargetVoltage, double rightTargetVoltage) {
        leftFlywheel.setVoltage(leftTargetVoltage);
        rightFlywheel.setVoltage(rightTargetVoltage);
    }

    public double getCalculatedVelocity() {
        if (flywheelVelocityMap != null) return flywheelVelocityMap.get(photonConstants.speakerDistance);
        return 2900;
    }

    public void populateFlywheelVelocityMap() {
        // TEST VALUES
        flywheelVelocityMap.put(1.12, 2600.0);
        flywheelVelocityMap.put(2.0, 2700.0);
        flywheelVelocityMap.put(3.0, 2800.0);
        flywheelVelocityMap.put(4.0, 2900.0);
    }
}