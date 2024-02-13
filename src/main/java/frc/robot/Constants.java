package frc.robot;

public class Constants {

    /**
     * CAN IDs
     */

    // Shooter
    public static final int TOP_SHOOTER_ID = 50;
    public static final int BOTTOM_SHOOTER_ID = 51;
    
    // Shooter Tilt/Angle
    public static final int ANGLE_MOTOR_ID = 52;
    public static final int ANGLE_ENCODER_ID = 53;

    // Intake

    // Conveyor
    public static final int SHOOTER_FEEDER_ID = 60;

    // Climbers

    

    /**
     * Constants for Angle Motor
     */
    public static final double SHOOTER_ANGLE_ZERO_OFFSET = 65.0;
    public static final double ANGLE_GEAR_RATIO = (36.0 / 12.0) * (79.0 / 10.0);
    public static final double ANGLE_ENCODER_OFFSET = 0.454834 ;
    public static final double ANGLE_PID_kP = 130.0;
    public static final double ANGLE_PID_kI = 60.0;
    public static final double ANGLE_PID_kD = 0.0;
    public static final double ANGLE_MOTIONMAGIC_ACC = 1.5;
    public static final double ANGLE_MOTIONMAGIC_CV = 6.0;
    public static final double ANGLE_MOTOR_CURRENT_LIMIT = 30.0; // Amps
    public static final double ANGLE_MOTOR_CURRENT_LIMIT_HOME = 1.0; // Amps


    /**
     * Shooter Location Constants
     * Angles measured in degrees
     */
    public static final double LOCATION_SUBWOOFER = 65.0;
    public static final double LOCATION_TRUSS = 88;
    public static final double LOCATION_WING = 75.0;
    public static final double LOCATION_TEST = 100.0;
    public static final double LOCATION_HOME = 67.0;

     
}
