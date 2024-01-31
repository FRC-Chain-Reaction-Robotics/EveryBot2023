package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.kauailabs.navx.frc.AHRS;
//import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.NeutralMode;
//import com.ctre.phoenix.motorcontrol.can.SSPX; // Ron i swear i will delete this line if its the last thing i do
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


/*basically have to create a PID loop */
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;




import java.io.IOException;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
// import frc.robot.lib.PhotonCameraWrapper;
import frc.robot.lib.NavX;

public class Tank extends SubsystemBase{
 /*
   * Autonomous selection options.
   */
  private static final String kNothingAuto = "do nothing";
  private static final String kConeAuto = "cone";
  private static final String kCubeAuto = "cube";
  
  /*
   * HowdyBot PID code
   */
 
//   private final SendableChooser<String> m_chooser = new SendableChooser<>();
  public static final double kMaxSpeed = 4466; //RPM
  public static SparkMaxPIDController leftController;
  public static SparkMaxPIDController rightController;
  public static double driveKp = .01;
  public static RelativeEncoder encoderLeft;
  public static RelativeEncoder encoderRight;
/*
   * Drive motor controller instances.
   *
   * Change the id's to match your robot.
   * Change kBrushed to kBrushless if you are using NEO's.
   * Use the appropriate other class if yo u are using different controllers.
   */
  CANSparkMax driveLeftLeader = new CANSparkMax(5, MotorType.kBrushless);
  CANSparkMax driveRightLeader = new CANSparkMax(3, MotorType.kBrushless); // id had problems
  CANSparkMax driveLeftFollower = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax driveRightFollower = new CANSparkMax(6, MotorType.kBrushless);
 
  

  SlewRateLimiter filter = new SlewRateLimiter(0.2);


  //CANSparkMax intake = new CANSparkMax(11, MotorType.kBrushless);// <- id here has problems


  NavX navX = new NavX();


  /**
   * The starter code uses the most generic joystick class.
   *
   * The reveal video was filmed using a logitech gamepad set to
   * directinput mode (switch set to D on the bottom). You may want
   * to use the XBoxController class with the gamepad set to XInput
   * mode (switch set to X on the bottom) or a different controller
   * that you feel is more comfortable.
   */
  XboxController j = new XboxController(1); // Was 0 but changed due to testing

  DifferentialDrive dt = new DifferentialDrive(new MotorControllerGroup(driveLeftLeader, driveLeftFollower), new MotorControllerGroup(driveRightLeader, driveRightFollower));

  /*
   * Magic numbers. Use these to adjust settings.
   */
  public static double output = 0.7;

  

  /**  
   * How many amps the intake can use while picking up
   */
  static final int INTAKE_CURRENT_LIMIT_A = 25;


  /**
   * How many amps the intake can use while holding
   */
  static final int INTAKE_HOLD_CURRENT_LIMIT_A = 5;


  /**
   * Percent output for intaking
   */
  static final double INTAKE_OUTPUT_POWER = 1.0;


  /**
   * Percent output for holding
   */
  static final double INTAKE_HOLD_POWER = 0.07;


 


  /**
   * Time to throw game piece in auto
   */
  static final double AUTO_THROW_TIME_S = 0.375;


  /**
   * Time to drive back in auto
   */
  static final double AUTO_DRIVE_TIME = 6.0;


  /**
   * Speed to drive backwards in auto
   */
  static final double AUTO_DRIVE_SPEED = -0.25;







  public Tank() {
    




    leftController = driveLeftLeader.getPIDController();
    rightController = driveRightLeader.getPIDController();

    encoderLeft = driveLeftLeader.getEncoder();
    encoderRight = driveRightLeader.getEncoder();

    encoderLeft.setPositionConversionFactor(Units.inchesToMeters(6*Math.PI));
    encoderRight.setPositionConversionFactor(Units.inchesToMeters(6*Math.PI));

    leftController.setFeedbackDevice(encoderLeft);
    rightController.setFeedbackDevice(encoderRight);

    leftController.setP(driveKp);
    rightController.setP(driveKp);
   
    /*
     * You will need to change some of these from false to true.
     *
     * In the setDriveMotors method, comment out all but 1 of the 4 calls
     * to the set() methods. Push the joystick forward. Reverse the motor
     * if it is going the wrong way. Repeat for the other 3 motors.
     */
     
    driveLeftLeader.setInverted(true);
    driveLeftFollower.setInverted(true);
    driveRightLeader.setInverted(false);
    driveRightFollower.setInverted(false);


    
   
   // intake.setInverted(false);
   // intake.setIdleMode(IdleMode.kBrake);
   
   driveLeftLeader.setIdleMode(IdleMode.kCoast);
   driveLeftFollower.setIdleMode(IdleMode.kCoast);
   driveRightLeader.setIdleMode(IdleMode.kCoast);
   driveRightFollower.setIdleMode(IdleMode.kCoast);
  }





  /**
   * Calculate and set the power to apply to the left and right
   * drive motors.
   *
   * @param forward Desired forward speed. Positive is forward.
   * @param turn    Desired turning speed. Positive is counter clockwise from
   *                above.
   */
 
  /*public void setDriveMotors(double left, double right) {
    SmartDashboard.putNumber("drive left power (%)", left);
    SmartDashboard.putNumber("drive right power (%)", right);


    // see note above in robotInit about commenting these out one by one to set
    // directions.

      
      This is the code in WPILib
      driveRightLeader.set(rightController.calculate(encoder.getDistance(), setpoint)); 
      
      //driveRightLeader.set(rightController.calculate(encoderLeft.getDistance(), -100));
      driveRightLeader.set(filter.calculate(right));  
      //driveLeftLeader.set(leftController.calculate(encoderRight.getDistance(), 100));
      driveLeftLeader.set(filter.calculate(left));


  
    
    
  } */
 


  

  public void setIdleMode(IdleMode idleMode){
    driveLeftLeader.setIdleMode(idleMode);
    driveLeftFollower.setIdleMode(idleMode);
    driveRightLeader.setIdleMode(idleMode);
    driveRightFollower.setIdleMode(idleMode);
  }
  /**
   * Set the arm output power.
   *
   * @param percent desired speed
   * @param amps current limit
   */
  /*public void setIntakeMotor(double percent, int amps) {
    intake.set(percent);
    intake.setSmartCurrentLimit(amps);
    SmartDashboard.putNumber("intake power (%)", percent);
    SmartDashboard.putNumber("intake motor current (amps)", intake.getOutputCurrent());
    SmartDashboard.putNumber("intake motor temperature (C)", intake.getMotorTemperature());
  } */

  public void drive(double speed, double rotation)
  {
    dt.arcadeDrive(output*(speed), output*(rotation));
  }

  public void resetEncoders()
  {
    encoderLeft.setPosition(0);
    encoderRight.setPosition(0);
  }

  public void goodMode()
  {
    driveLeftLeader.setSmartCurrentLimit(35, 35);
    driveLeftFollower.setSmartCurrentLimit(35, 35);
    driveRightLeader.setSmartCurrentLimit(35, 35);
    driveRightFollower.setSmartCurrentLimit(35, 35);


    output = 0.7;
  }

  public void evilMode()
  {
    driveLeftLeader.setSmartCurrentLimit(45, 45);
    driveLeftFollower.setSmartCurrentLimit(45, 45);
    driveRightLeader.setSmartCurrentLimit(45, 45);
    driveRightFollower.setSmartCurrentLimit(45, 45);

    output = 0.3;
  }

  public void fastMode()
  {
    driveLeftLeader.setSmartCurrentLimit(25, 25);
    driveLeftFollower.setSmartCurrentLimit(25, 25);
    driveRightLeader.setSmartCurrentLimit(25, 25);
    driveRightFollower.setSmartCurrentLimit(25, 25);
    output = 1;
  }
  
  public double getDistanceMeters()
  {
    return Math.max(encoderLeft.getPosition(),encoderRight.getPosition());
  }

  public double getHeading(){
    return navX.getYaw();
  }
}
