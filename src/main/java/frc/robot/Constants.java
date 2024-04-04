package frc.robot;

public class Constants {

    /**
     * CAN IDs
     */

    // Shooter
    public static final int SHOOTER_TOP_ID = 50;
    public static final int SHOOTER_BOTTOM_ID = 54;
    
    // Shooter Tilt/Angle
    public static final int ANGLE_MOTOR_ID = 52;
    public static final int ANGLE_ENCODER_ID = 53;

    // Intake
    public static final int INTAKE_TOP_ID = 59;
    public static final int INTAKE_BOTTOM_ID = 61;
    public static final int INTAKE_PIVOT_RIGHT_ID = 56;
    public static final int INTAKE_PIVOT_LEFT_ID = 57;

    // Conveyor
    public static final int SHOOTER_FEEDER_ID = 60;
    public static final int CONVEYOR_TOP_ID = 62;
    public static final int CONVEYOR_BOTTOM_ID = 58;

    // Climbers
    public static final int CLIMBER_RIGHT_ID = 55;
    public static final int CLIMBER_LEFT_ID = 51;


    /**
     * Misc Channel Configs
     */

    public static final int CONVEYOR_NOTE_SENSOR_CHANNEL = 9;
    public static final int LED_PORT = 0;

    

    /**
     * Constants for Angle Motor
     */
    public static final double SHOOTER_ANGLE_ZERO_OFFSET = 72.0;
    public static final double ANGLE_GEAR_RATIO = (36.0 / 12.0) * (79.0 / 10.0);
    public static final double ANGLE_ENCODER_OFFSET = 0.289795 ;
    public static final double ANGLE_PID_kP = 130.0;
    public static final double ANGLE_PID_kI = 60.0;
    public static final double ANGLE_PID_kD = 0.0;
    public static final double ANGLE_MOTIONMAGIC_ACC = 1.5;
    public static final double ANGLE_MOTIONMAGIC_CV = 6.0;
    public static final double ANGLE_MOTOR_CURRENT_LIMIT = 20.0; // Amps
    public static final double ANGLE_MOTOR_CURRENT_LIMIT_HOME = 3.0; // Amps
    public static final double SHOOTER_ANGLE_DEFAULT_TOLERANCE = 1.5;


    /**
     * Intake Position PID
     */

    public static final double INTAKE_kP = .65;
    public static final double INTAKE_kI = .005;
    public static final double INTAKE_kD = .015;
    public static final double INTAKE_kV = 0.0;
    public static final double INTAKE_MOTIONMAGIC_ACC = 160;
    public static final double INTAKE_MOTIONMAGIC_CV = 30;
    public static final double INTAKE_CURRENT_LIMIT = 30;
    public static final double INTAKE_DOWN_POS = 8.05;
    public static final double INTAKE_POS_DELTA = 7.85;
    public static final double INTAKE_UP_POS = .167;
    public static final double INTAKE_AMP_POS = 2.0;
    public static final double INTAKE_ROTOR_TO_MECHANISM_RATIO = 21.667;


    /**
     * Shooter Constants
     */
    public static final double SHOOTER_kP = 0.2;
    public static final double SHOOTER_kI = 0.2;
    public static final double SHOOTER_kD = 0.0;
    public static final double SHOOTER_kV = 0.11;
    public static final double SHOOTER_MMAcceleration = 2300;

    public static final double SHOOTER_RPM_TOLERANCE_PERCENT = .05; // 5%
    public static final double SHOOTER_CURRENT_LIMIT = 50.0;

    public static final double SUBWOOFER_TOP_RPM = 2850;
    public static final double SUBWOOFER_BOTTOM_RPM = 2850;
    
    // Changing constants to up spped for a consistent shot
    public static final double AMP_TOP_RPM = 750; //650 -> 750
    public static final double AMP_BOTTOM_RPM = 750; //500 -> 750

    public static final double TRUSS_TOP_RPM = 2850;
    public static final double TRUSS_BOTTOM_RPM = 3100;
    
    public static final double AUTO_SHOOT_NOTE_OUT_DELAY = 0.1; // Seconds

    /*
     * Conveyor Constants
     */

    public static final double CONVEYOR_TOP_WHEEL_DIA = 3.0;
    public static final double CONVEYOR_TOP_GEAR_RATIO = 2.0;
    public static final double CONVEYOR_BOTTOM_WHEEL_DIA = 2.0;
    public static final double CONVEYOR_BOTTOM_GEAR_RATIO = 5.0;

    /**
     * Shooter Location Constants
     * Angles measured in degrees
     */
    public static final double LOCATION_SUBWOOFER = 72.5;
    public static final double LOCATION_TRUSS = 97.0;
    public static final double LOCATION_WING = 79.0;
    public static final double LOCATION_TEST = 100.0;
    public static final double LOCATION_HOME = 71;

    /**
     * Climber Constants
     */

    public static final double CLIMBER_CURRENT_LIMIT = 30.0;
    public static final double AUTO_DISABLE_BRAKE_TIME_SEC = 30.0;

    /**
     * Drive Current Limits
     */
    public static final double DRIVE_CURRENT_LIMIT = 65.0;
    public static final double STEER_CURRENT_LIMIT = 30.0;


    /**
     * General Constants
     */
    public static final double NEO_MAX_SPEED = 5676;
    public static final double NEO550_MAX_SPEED = 11000;

     
}
