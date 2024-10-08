package frc.robot.utils;

import java.util.Arrays;

import com.fasterxml.jackson.annotation.JsonRawValue;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.REVLibError;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;

public final class Utils {

  private static final ObjectMapper objectMapper = new ObjectMapper();

  @JsonRawValue
  public static String objectToJson(Object o) {
    try {
      return objectMapper.writeValueAsString(o);
    } catch (Exception ex) {
      return "{}";
    }
  }

  public static <T> T getValueForAlliance(T blueValue, T redValue) {
    return Robot.getAlliance() == Alliance.Blue ? blueValue : redValue;
  }

  public static boolean isValueInRange(double value, double minValue, double maxValue) {
    return value >= minValue && value <= maxValue;
  }

  public static double squareInput(double input, double deadband) {
    double deadbandInput = MathUtil.applyDeadband(input, deadband);
    return (deadbandInput * deadbandInput) * Math.signum(input);
  }

  public static double getVoltsToPsi(double sensorVoltage, double supplyVoltage) {
    return 250 * (sensorVoltage / supplyVoltage) - 25;
  }

  public static double wrapAngle(double angle) {
    return MathUtil.inputModulus(angle, -180, 180);
  }

  public static double getYawToPose(Pose2d robotPose, Pose2d targetPose) {
    Translation2d translation = targetPose.relativeTo(robotPose).getTranslation();
    return wrapAngle(new Rotation2d(translation.getX(), translation.getY()).getDegrees());
  }

  public static double getPitchToPose(Pose3d robotPose, Pose3d targetPose) {
    return Math.toDegrees(Math.atan2(targetPose.minus(robotPose).getZ(), getDistanceToPose(robotPose.toPose2d(), targetPose.toPose2d())));
  }

  public static double getDistanceToPose(Pose2d robotPose, Pose2d targetPose) {
    return robotPose.getTranslation().getDistance(targetPose.getTranslation());
  }

  public static double getLinearInterpolation(double[] xs, double[] ys, double x) {
    double[] dx = new double[xs.length - 1];
    double[] dy = new double[xs.length - 1];
    double[] slope = new double[xs.length - 1];
    double[] intercept = new double[xs.length - 1];
    for (int i = 0; i < xs.length - 1; i++) {
      dx[i] = xs[i + 1] - xs[i];
      dy[i] = ys[i + 1] - ys[i];
      slope[i] = dy[i] / dx[i];
      intercept[i] = ys[i] - xs[i] * slope[i];
    }
    double y;
    if ((x > xs[xs.length - 1]) || (x < xs[0])) {
      y = Double.NaN;
    }
    else {
      int loc = Arrays.binarySearch(xs, x);
      if (loc < -1) {
        loc = -loc - 2;
        y = slope[loc] * x + intercept[loc];
      }
      else {
        y = ys[loc];
      }
    }
    return y;
  }

  public static void enableSoftLimits(CANSparkBase controller, boolean isEnabled) {
    controller.enableSoftLimit(SoftLimitDirection.kForward, isEnabled);
    controller.enableSoftLimit(SoftLimitDirection.kReverse, isEnabled);
  }

  /*public static void validateREVLib(REVLibError error) {
    Timer.delay(0.001);
    if (error != REVLibError.kOk) {
      Logger.error("REVLibError: " + error.toString());
    }
  }

  public static void validateREVLib(boolean predicate) {
    Timer.delay(0.001);
    if (!predicate) {
      Logger.error("REVLibError: set parameter call failed validation!");
    }
  }*/
}