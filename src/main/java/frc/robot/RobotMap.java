package frc.robot;

public abstract class RobotMap {

    public static final int kFrontLeftDriveMotorPort = 23;
    public static final int kRearLeftDriveMotorPort = 20;
    public static final int kFrontRightDriveMotorPort = 10;
    public static final int kRearRightDriveMotorPort = 16;

    public static final int kFrontLeftTurningMotorPort = 12;
    public static final int kRearLeftTurningMotorPort = 8;
    public static final int kFrontRightTurningMotorPort = 7;
    public static final int kRearRightTurningMotorPort = 5;

    public static final int[] kFrontLeftTurningEncoderPorts = new int[] {0, 1};
    public static final int[] kRearLeftTurningEncoderPorts = new int[] {2, 3};
    public static final int[] kFrontRightTurningEncoderPorts = new int[] {4, 5};
    public static final int[] kRearRightTurningEncoderPorts = new int[] {5, 6};

    public static final int[] kFrontLeftDriveEncoderPorts = new int[] {7, 8};
    public static final int[] kRearLeftDriveEncoderPorts = new int[] {9, 10};
    public static final int[] kFrontRightDriveEncoderPorts = new int[] {11, 12};
    public static final int[] kRearRightDriveEncoderPorts = new int[] {13, 14};

    // Non-Drive Motors
    public static final int INDEXER_MOTOR = 1;
    public static final int INDEXER_STOPPER_MOTOR = 2;
    public static final int FLYWHEEL_MOTOR = 13;
    public static final int FLYWHEEL_MOTOR_2 = 14;
    public static final int SINGULATOR_MOTOR = 15;
    public static final int COLLECTOR_MOTOR = 4;
    public static final int SHOOTER_HOOD_MOTOR = 3;

    public static final int SENSOR_ONE = 0;
    public static final int SENSOR_TWO = 1;

    // Pigeon
    public static final int PIGEON = 4;

    // Controllers
    public static final int DRIVER_CONTROLLER_ID = 0;
    public static final int AUX_CONTROLLER_ID = 1;
}
