package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {
  public static final class Drive {
    public static final int FRONT_LEFT_ID = 1;
    public static final int FRONT_RIGHT_ID = 2;
    public static final int BACK_LEFT_ID = 3;
    public static final int BACK_RIGHT_ID = 4;
    public static final int GYRO_ID = 0;

    public static final double TRACK_WIDTH = 0.6; // Meters
    public static final double WHEEL_RADIUS = Units.inchesToMeters(3);
    public static final double GEAR_RATIO = 10.71;
    public static final double POSITION_CONVERSION = 2 * Math.PI * WHEEL_RADIUS / GEAR_RATIO;
    public static final double VELOCITY_CONVERSION = POSITION_CONVERSION / 60;

    public static final double ACCELERATION_LIMIT = 2.0; // Percent voltage per second
    public static final double MAX_SPEED = 0.8; // Percent voltage
    public static final double MAX_TURN = 0.4; // Percent voltage
    public static final double ANGLE_TOLERANCE = Units.degreesToRadians(10);
    public static final double TURNING_kP = 0;
    public static final double TURNING_kI = 0;
    public static final double TURNING_kD = 0;
  }
}
