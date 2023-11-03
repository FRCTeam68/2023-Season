package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.COTSFalconSwerveConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final double BEAM_BALANACED_DRIVE_KP = 0.05; // P (Proportional) constant of a PID loop 0.015
    public static final double BEAM_BALANCED_GOAL_DEGREES = 0;
    public static final double BEAM_BALANCED_ANGLE_TRESHOLD_DEGREES = 6.8;
    public static final double BALANCEDMAXSPEED = 3;
    public static final double BALANCED_OFFESET = 2.3;

    public static final class Swerve {
       
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
            COTSFalconSwerveConstants.SDSMK3(6.63);

        public static final double gearRatio = 6.63;

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(17); //TODO: This must be tuned to specific robot 
        public static final double wheelBase = Units.inchesToMeters(22); //TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; 
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; 

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { 
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 15;
            public static final double dobOffset = -0.309814;  //257.50);  //18.3691+180 //295.576

        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 16;
            public static final double dobOffset = -0.692871;  //257.50);  //18.3691+180 //295.576

        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 17;
            public static final double dobOffset = -0.682129+0.020;  //257.50);  //18.3691+180 //295.576

        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 18;
            public static final double dobOffset = -0.38916;  //257.50);  //18.3691+180 //295.576
        }
    }

    public static final int NAVX = 19;

    public static final String MANIP_CANBUS = "DADbus";

    public static final double DEADBAND = 0.1;

    public class ARM {
        public static final int EXTENDMOTOR = 13;
		public static final int ROTATEMOTOR = 12;
        public static final int EXTENDENCODER = 20;
        public static final int ROTATEENCODER = 21;

        public static final double MAX_MANUAL_SUPPLY_VOLTAGE = 11.0;
    }

    public final static class INTAKE {
        public static final int INTAKE_MOTOR = 14;

        public static final double MAX_SUPPLY_VOLTAGE = 11.0;
    }

    public static final class AutoConstants { 
        public static final double kMaxSpeed = 3; // m / s
        public static final double kMaxAcceleration = 3; // m / s^2
        public static final double kMaxAngularSpeed = Math.PI; // Radians / s
        public static final double kMaxAngularAcceleration = Math.PI; // Radians / s^2
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeed, kMaxAngularAcceleration);
    }
}
