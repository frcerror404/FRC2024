package frc.robot;

public class Constants {

    /**
     * CAN IDs
     */

    // Shooter
    public static final int SHOOTER_TOP_ID = 50;
    public static final int SHOOTER_BOTTOM_ID = 51;
    
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
    public static final int CLIMBER_LEFT_ID = 54;


    

    /**
     * Constants for Angle Motor
     */
    public static final double SHOOTER_ANGLE_ZERO_OFFSET = 65.0;
    public static final double ANGLE_GEAR_RATIO = (36.0 / 12.0) * (79.0 / 10.0);
    public static final double ANGLE_ENCODER_OFFSET = 0.448730 ;
    public static final double ANGLE_PID_kP = 130.0;
    public static final double ANGLE_PID_kI = 60.0;
    public static final double ANGLE_PID_kD = 0.0;
    public static final double ANGLE_MOTIONMAGIC_ACC = 1.5;
    public static final double ANGLE_MOTIONMAGIC_CV = 6.0;
    public static final double ANGLE_MOTOR_CURRENT_LIMIT = 20.0; // Amps
    public static final double ANGLE_MOTOR_CURRENT_LIMIT_HOME = 5.0; // Amps


    /**
     * Intake Position PID
     */

    public static final double INTAKE_kP = .65;
    public static final double INTAKE_kI = .005;
    public static final double INTAKE_kD = .03;
    public static final double INTAKE_kV = 0.0;
    public static final double INTAKE_MOTIONMAGIC_ACC = 160;
    public static final double INTAKE_MOTIONMAGIC_CV = 30;
    public static final double INTAKE_CURRENT_LIMIT = 30;
    public static final double INTAKE_DOWN_POS = 8.05;
    public static final double INTAKE_POS_DELTA = 7.85;
    public static final double INTAKE_UP_POS = .167;
    public static final double INTAKE_AMP_POS = 2.0;
    public static final double INTAKE_ROTOR_TO_MECHANISM_RATIO = 21.667;

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
    public static final double LOCATION_SUBWOOFER = 65.0;
    public static final double LOCATION_TRUSS = 88;
    public static final double LOCATION_WING = 79.0;
    public static final double LOCATION_TEST = 100.0;
    public static final double LOCATION_HOME = 70.0;

    /**
     * Climber Constants
     */

    public static final double CLIMBER_CURRENT_LIMIT = 30.0;
    public static final double AUTO_DISABLE_BRAKE_TIME_SEC = 10.0;


    /**
     * General Constants
     */
    public static final double NEO_MAX_SPEED = 5676;
    public static final double NEO550_MAX_SPEED = 11000;

     
}
