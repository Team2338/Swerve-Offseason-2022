package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.lib.Gains;

public final class Constants {

    public static final class Drivetrain {
        //public static final double DRIVE_WHEEL_RADIUS = 0.05; // meters? Must be unit of velocity

        public static final boolean kFrontLeftTurningEncoderReversed = false; //false
        public static final boolean kRearLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kRearRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveMotorReversed = true;
        public static final boolean kRearLeftDriveMotorReversed = true;
        public static final boolean kFrontRightDriveMotorReversed = false;
        public static final boolean kRearRightDriveMotorReversed = false;

        public static final boolean kFrontLeftTurningMotorReversed = true;
        public static final boolean kRearLeftTurningMotorReversed = false;
        public static final boolean kFrontRightTurningMotorReversed = false;
        public static final boolean kRearRightTurningMotorReversed = false;

        public static final double kFrontLeftOffset = -3.468330561409435;
        public static final double kRearLeftOffset = -6.18347655596702;
        public static final double kFrontRightOffset = -4.428602534625846;
        public static final double kRearRightOffset = -0.739378739760879;

        public static final double kTrackWidth = 0.4699;
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = 0.4699;
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics =
                new SwerveDriveKinematics(
                        new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // x was +, y was +
                        new Translation2d(kWheelBase / 2, kTrackWidth / 2), // x was +, y was -
                        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2), // x was -, y was +
                        new Translation2d(-kWheelBase / 2, kTrackWidth / 2)); // x was -, y was -

        public static final boolean kGyroReversed = false;

        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or theoretically
        // for *your* robot's drive.
        // The RobotPy Characterization Toolsuite provides a convenient tool for obtaining these
        // values for your robot.
        public static final double ksVolts = 0.126;
        public static final double kvVoltSecondsPerMeter = 0.0185;
        public static final double kaVoltSecondsSquaredPerMeter = 0.00227;

        public static final double kMaxDriveRPM = 4800;

        public static final double kMaxSpeedMetersPerSecond = kMaxDriveRPM *
                (Math.PI * Constants.ModuleConstants.kWheelDiameterMeters) /
                (60.0 * Constants.ModuleConstants.kGearRatio);
        public static double kMaxAccelerationMetersPerSecondSquared = 2;// needs real number

    }

    public static class ModuleConstants {
        public static final double kMaxModuleAngularSpeedRadiansPerSecond = 6 * (2 * Math.PI); //6
        public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 6 * (2 * Math.PI); //7

        public static final double kEncoderCPR = 4096.0; //1024
        public static final double kFalconEncoderCPR = 2048;
        public static final double kWheelDiameterMeters = 0.10338;
        public static final double kDriveEncoderDistancePerPulse =
                // Assumes the encoders are directly mounted on the wheel shafts
                (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

        public static final double kTurningEncoderDistancePerPulse =
                // Assumes the encoders are on a 1:1 reduction with the module shaft.
                (2 * Math.PI) / (double) kEncoderCPR;

        public static final double kPModuleTurningController = 1.0; // 1

        public static final double kPModuleDriveController = 0.3; // 1

        public static final double kGearRatio = 46080.0 / 6720.0;
        //public static final double kGearRatio = 6720.0 / 46080.0; // WRONG
    }
    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 2;
        public static final double kPYController = 2;
        public static final double kPThetaController = 2.2;

        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static class Shooter {
        public static final double kP = 0.0007;
        public static final double kF = 0.000175;

        public static final int GREEN_ZONE = 1;
        public static final int YELLOW_ZONE = 2;
        public static final int BLUE_ZONE = 3;
        public static final int RED_ZONE = 4;

        public static final int HOOD_POS_GREEN = 3020;
        public static final int HOOD_POS_YELLOW = 7500;
        public static final int HOOD_POS_BLUE = 10500;
        public static final int HOOD_POS_RED = 11750;

        public static final int RPM_GREEN = 3400;
        public static final int RPM_YELLOW = 4500;
        public static final int RPM_BLUE = 4500;
        public static final int RPM_RED = 4900;

        // feedforward of 2nd flywheel motor
        public static final double FF_GREEN = 0.6;
        public static final double FF_YELLOW = 0.78;
        public static final double FF_BLUE = 0.79;
        public static final double FF_RED = 1.0;
    }

    public static class Hood {
        /**
         * Which PID slot to pull gains from. Starting 2018, you can choose from
         * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
         * configuration.
         */
        public static final int kSlotIdx = 0;

        /**
         * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For
         * now we just want the primary one.
         */
        public static final int kPIDLoopIdx = 0;

        /**
         * Set to zero to skip waiting for confirmation, set to nonzero to wait and
         * report to DS if action fails.
         */
        public static final int kTimeoutMs = 30;

        /* Choose so that Talon does not report sensor out of phase */
        public static boolean kSensorPhase = false;

        /**
         * Choose based on what direction you want to be positive,
         * this does not affect motor invert.
         */
        public static boolean kMotorInvert = true;

        // F-gain = ([Percent Output] x 1023) / [Velocity]
        public static double kF = (0.25 * 1023) / 28800;

        /**
         * Gains used in Positon Closed Loop, to be adjusted accordingly
         * Gains(kp, ki, kd, kf, izone, peak output);
         */
        public static final Gains kGains = new Gains(0.01023 * 16, 0.0, 0.0, kF, 0, 0.5);
        // 0.15, 0.0, 1.0, 0.0, 0, 1.0

        /* Motion Magic
        public static final double ALLOWABLE_ERROR = 500;
        public static final double P = 0.5;
        public static final double I = 0;
        public static final double D = 0;
        public static final double F = 0.2;

        public static final int MAX_VELOCITY = 1000; // Hood velocity (ticks/100ms)
        public static final int MAX_ACCELERATION = 1000; // Hood acceleration (ticks/100ms/s)
        public static final double GRAV_FEED_FORWARD = 0;
        public static final int REV_MAX_VELOCITY = MAX_VELOCITY;
        public static final double REV_F = 0.1;
        public static final double REV_GRAV_FEED_FORWARD = 0;

        public static final int MAX_POS = 10000;
        public static final int MIN_POS = 0;*/
    }
}
