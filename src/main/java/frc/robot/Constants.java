package frc.robot;

import java.io.IOException;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static final class Controllers {
        public static final int kDriverControllerPort = 0; 
        public static final int kOperatorControllerPort = 1; 
        public static final double kDeadband = 0.1; 
    }

    /*
    public static final class Swerve {
        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        // Measure from CANcoder to CANcoder, as they are the center of the wheels
        // Change these!

        // measured 3/21/23
        public static final double kTrackWidth = Units.inchesToMeters(13.3);
        // Distance between centers of right and left wheels on robot
        // measured 3/21/23
        public static final double kWheelBase = Units.inchesToMeters(23.5);
        // Distance between front and back wheels on robot
       
        //Calculates to approximately 4.47596143982 meters/sec
        public static final double kMaxSpeedMetersPerSecond = (SwerveModule.kDrivingMotorFreeSpeedRps / 
            SwerveModule.kDrivingMotorReduction) * 
            SwerveModule.kWheelDiameterMeters * 
            Math.PI;
        
        //Calculates to Approximately 18.5429440372 rads/sec (Too much??)
        public static final double kMaxAngularSpeed = kMaxSpeedMetersPerSecond/Math.hypot(kTrackWidth/2.0, kWheelBase/2.0); // radians per second
        
        public static final double kMaxAccel = kMaxSpeedMetersPerSecond * 2;

        public static final double kMaxAngularAccel = kMaxAccel/Math.hypot(kTrackWidth/2.0, kWheelBase/2.0);
        // Chassis configuration
        

        ////Translation2d's are the x and y coordinates relative to your gyro. In this instance, 
        ////the four Translation2d objects are the coordinates of your four swerve modules
        //// x is front and back and y is left and right
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
        // Swerve Module IDs

        //Front Left Module
        public static final int kFrontLeftDrivingCanId = 8;
        public static final int kFrontLeftTurningCanId = 5;
        public static final int kFrontLeftCanCoderId = 14;
        public static final double kFrontLeftChassisAngularOffset = Math.toRadians(277.725);   //   epic
        
        //Front Right Module
        public static final int kFrontRightDrivingCanId = 4; 
        public static final int kFrontRightTurningCanId = 2;
        public static final int kFrontRightCanCoderId = 61;
        public static final double kFrontRightChassisAngularOffset = Math.toRadians(52.207);
        
         //Back Left Module
        public static final int kBackLeftDrivingCanId = 10;
        public static final int kBackLeftTurningCanId = 1;
        public static final int kBackLeftCanCoderId = 60;
        public static final double kBackLeftChassisAngularOffset = Math.toRadians(269.824); // manually zeroed
        
        //Back Right Module
        public static final int kBackRightDrivingCanId = 7;
        public static final int kBackRightTurningCanId = 3;
        public static final int kBackRightCanCoderId = 62;
        public static final double kBackRightChassisAngularOffset = Math.toRadians(337.676);

        public static final boolean kGyroReversed = false;
      }
      */

      /*
      public static final class SwerveModule {
        // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
        // This changes the drive speed of the module (a pinion gear with more teeth will result in a
        // robot that drives faster).
        public static final int kDrivingMotorPinionTeeth = 14;
    
        // Invert the turning encoder, since the output shaft rotates in the opposite direction of
        // the steering motor in the MAXSwerve Module.
        public static final boolean kTurningEncoderInverted = true;
    
        // Calculations required for driving motor conversion factors and feed forward
        public static final double kNeoMotorFreeSpeedRpm = 5676;

        public static final double kDrivingMotorFreeSpeedRps = kNeoMotorFreeSpeedRpm / 60.0;
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4); // 3 : 0.0762; // 3.75 : 0.09525; //3.8 : 0.09652; // 4 : 0.1016; //Units.inchesToMeters(4.0);
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
        public static final double kDrivingMotorReduction = (50.0 * 17.0 * 45.0) / (kDrivingMotorPinionTeeth * 27.0 * 15.0);//
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
            /kDrivingMotorReduction;
    
        public static final double kSteeringMotorReduction = 12.8;
    
        public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
            / kDrivingMotorReduction; // meters
        public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
            / kDrivingMotorReduction) / 60.0; // meters per second
    
        public static final double kTurningEncoderPositionFactor = (2 * Math.PI) / kSteeringMotorReduction; // radians
        public static final double kTurningEncoderVelocityFactor = ((2 * Math.PI) / kSteeringMotorReduction) / 60.0; // radians per second
    
        public static final double kTurningEncoderPositionPIDMinInput = -Math.PI; // radians
        public static final double kTurningEncoderPositionPIDMaxInput = Math.PI; // radians
    
        public static final double kTurningAnalogPositionFactor = (Math.PI * 2 / 3.3);
        public static final double kTurningAnalogVelocityFactor = (kTurningAnalogPositionFactor/60);
        
        public static final double kDrivingP = 0.15;
        public static final double kDrivingI = 0;
        public static final double kDrivingD = 0.015;
        public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
        public static final double kDrivingMinOutput = -1;
        public static final double kDrivingMaxOutput = 1;
    
        public static final double kTurningP = 2.295;
        public static final double kTurningI = 0;
        public static final double kTurningD = 0.2295;
        public static final double kTurningFF = 0;
        public static final double kTurningMinOutput = -1;
        public static final double kTurningMaxOutput = 1;
    
        public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
        public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;
    
        public static final int kDrivingMotorCurrentLimit = 40; // amps
        public static final int kTurningMotorCurrentLimit = 30; // amps
      }
      */

      /*
      public static final class Vision {
        public static AprilTagFieldLayout kAprilTagFieldLayout = null;
        static {
            try {
                kAprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    */

    /*
    public static final String kCameraName = "gloworm";
    public static final Transform3d kRobotToCamera =
        new Transform3d(
            new Translation3d(0.5, 0.0, 0.5),
            new Rotation3d(0, 0,0)); 
    */

}