// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;


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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;


import edu.wpi.first.math.controller.PIDController;


import frc.robot.lib.DataLog;

public class Robot extends TimedRobot {
  private RobotContainer m_robotContainer;
  private Command m_autonomousCommand;
  private String m_autoSelected;
  

  /*
   * HowdyBot PID code
   */

  private final SendableChooser<String> m_chooser = new SendableChooser<>();
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
   * Use the appropriate other class if you are using different controllers.
   */
  CANSparkMax driveLeftLeader = new CANSparkMax(5, MotorType.kBrushless);
  CANSparkMax driveRightLeader = new CANSparkMax(3, MotorType.kBrushless); // id had problems
  CANSparkMax driveLeftFollower = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax driveRightFollower = new CANSparkMax(6, MotorType.kBrushless);
 
  

  SlewRateLimiter filter = new SlewRateLimiter(0.2);
  /*
   * Mechanism motor controller instances.
   *
   * Like the drive motors, set the CAN id's to match your robot or use different
   * motor controller classses (TalonFX, TalonSRX, Spark, Spark2SP) to match your
   * robot.
   *
   * The arm is a NEO on Everybud.
   * The intake is a NEO 550 on Everybud.
   */
  CANSparkMax armLeader = new CANSparkMax(7, MotorType.kBrushless); // <- id here has problems
  CANSparkMax armFollower = new CANSparkMax(4, MotorType.kBrushless); // might have problems


  //CANSparkMax intake = new CANSparkMax(11, MotorType.kBrushless);// <- id here has problems


 


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


  /**
   * How many amps the arm motor can use.
   */
  static final int ARM_CURRENT_LIMIT_A = 20;


  /**
   * Percent output to run the arm up/down at
   */
  static final double ARM_OUTPUT_POWER = 0.35;


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
   * Time to extend or retract arm in auto
   */
  static final double ARM_EXTEND_TIME_S = 2.0;


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



  /**
   * This method is run once when the robot is first started up.
   */
  @Override
  public void robotInit() {
    DataLog.start();
    m_robotContainer = new RobotContainer();    
  }

    /*m_chooser.setDefaultOption("do nothing", kNothingAuto);
    m_chooser.addOption("cone and mobility", kConeAuto);
    m_chooser.addOption("cube and mobility", kCubeAuto);
    SmartDashboard.putData("Auto choices", m_chooser);




    leftController = driveLeftLeader.getPIDController();
    rightController = driveRightLeader.getPIDController();

    encoderLeft = driveLeftLeader.getEncoder();
    encoderRight = driveRightLeader.getEncoder();


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
     
   /* driveLeftLeader.setInverted(true);
    driveLeftFollower.setInverted(true);
    driveRightLeader.setInverted(false);
    driveRightFollower.setInverted(false);


    /*
     * Set the arm and intake to brake mode to help hold position.
     * If either one is reversed, change that here too. Arm out is defined
     * as positive, arm in is negative.
     */
    /*armLeader.setInverted(true);
    armLeader.setIdleMode(IdleMode.kBrake);
    armLeader.setSmartCurrentLimit(ARM_CURRENT_LIMIT_A);
   
    armFollower.setIdleMode(IdleMode.kBrake);
    armFollower.setSmartCurrentLimit(ARM_CURRENT_LIMIT_A);
    armFollower.follow(armLeader,true); */
   
   // intake.setInverted(false);
   // intake.setIdleMode(IdleMode.kBrake);


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
 


  /**
   * Set the arm output power. Positive is out, negative is in.
   *
   * @param percent -1 to 1
   */
  public void setArmMotor(double percent) {
    armLeader.set(percent);
    // armFollower.set(percent); // used to test a troubleshoot
    System.out.println(percent);
    SmartDashboard.putNumber("arm power (%)", percent*100);
    SmartDashboard.putNumber("arm motor current (amps)", armLeader.getOutputCurrent());
    SmartDashboard.putNumber("arm motor temperature (C)", armLeader.getMotorTemperature());
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
    // DataLog.start();
    // m_robotContainer = new RobotContainer();    
  



  /**
   * This method is called every 20 ms, no matter the mode. It runs after
   * the autonomous and teleop specific period methods.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Time (seconds)", Timer.getFPGATimestamp());
    CommandScheduler.getInstance().run();
  }



  @Override
  public void autonomousInit() {
    //might want to change kBrake to kCoast
    // stem gals: changed from kCoast to kBrake
    
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    //System.out.println("Auto selected: " + m_autoSelected);

    if (m_autonomousCommand != null)
      m_autonomousCommand.schedule();
    


  }


  @Override
  public void autonomousPeriodic() {
    /*if (m_autoSelected == kNothingAuto) {
      setArmMotor(0.0);
      setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
      //setDriveMotors(0.0, 0.0);
      setDriveMotors(-0.8,0.8); // added during start of ntx day 2
      return;
    } */


  /*   double timeElapsed = Timer.getFPGATimestamp() - autonomousStartTime;


    if (timeElapsed < ARM_EXTEND_TIME_S) {
      setArmMotor(ARM_OUTPUT_POWER);
      setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
      setDriveMotors(0.0, 0.0);
    } else if (timeElapsed < ARM_EXTEND_TIME_S + AUTO_THROW_TIME_S) {
      setArmMotor(0.0);
      setIntakeMotor(autonomousIntakePower, INTAKE_CURRENT_LIMIT_A);
      setDriveMotors(0.0, 0.0);
    } else if (timeElapsed < ARM_EXTEND_TIME_S + AUTO_THROW_TIME_S + ARM_EXTEND_TIME_S) {
      setArmMotor(-ARM_OUTPUT_POWER);
      setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
      setDriveMotors(0.0, 0.0);
    } else if (timeElapsed < ARM_EXTEND_TIME_S + AUTO_THROW_TIME_S + ARM_EXTEND_TIME_S + AUTO_DRIVE_TIME) {
      setArmMotor(0.0);
      setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
      setDriveMotors(AUTO_DRIVE_SPEED, 0.0);
    } else {
      setArmMotor(0.0);
      setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
      setDriveMotors(0.0, 0.0);
    } */
  }
   
  /**
   * Used to remember the last game piece picked up to apply some holding power.
   */


  @Override
  public void teleopInit() { // changed from kCoast
    if(m_autonomousCommand != null)
    {
      m_autonomousCommand.cancel();
    }
  }


  @Override
  public void teleopPeriodic() {
    //if (j.getRawButton(0) System.out.println(0);


    /*double armPower;
    if (j.getRawButton(5)) { // might be 6 or 7
      // lower the arm
      armPower = -ARM_OUTPUT_POWER;
      System.out.println("Lowering the arm");
    } else if (j.getRawButton(6)) { // might be 4 or 5
      // raise the arm
      armPower = ARM_OUTPUT_POWER;
      System.out.println("Raising the arm");
    } else {
      // do nothing and let it sit where it is
      armPower = 0.0;
    }
    setArmMotor(armPower);
   
 
    double intakePower;
    int intakeAmps;
    if (j.getRawAxis(3) > 0.7) { // right trigger
      // cube in or cone out
      intakePower = INTAKE_OUTPUT_POWER;
      intakeAmps = INTAKE_CURRENT_LIMIT_A;
      lastGamePiece = CUBE;
    } else if (j.getRawAxis(2) > 0.7) { // left trigger
      // cone in or cube out
      intakePower = -INTAKE_OUTPUT_POWER;
      intakeAmps = INTAKE_CURRENT_LIMIT_A;
      lastGamePiece = CONE;
    } else if (lastGamePiece == CUBE) {
      intakePower = INTAKE_HOLD_POWER;
      intakeAmps = INTAKE_HOLD_CURRENT_LIMIT_A;
    } else if (lastGamePiece == CONE) {
      intakePower = -INTAKE_HOLD_POWER;
      intakeAmps = INTAKE_HOLD_CURRENT_LIMIT_A;
    } else {
      intakePower = 0.0;
      intakeAmps = 0;
    }
    setIntakeMotor(intakePower, intakeAmps); */


    /*
     * Negative signs here because the values from the analog sticks are backwards
     * from what we want. Forward returns a negative when we want it positive.
     */
    // left stick up & down, right stick up and downs
    
    
   
   
    //setDriveMotors(-j.getRawAxis(1),j.getRawAxis(5)); //original working


    /* .7 is the constant that controls the speed of the robot, if you want to increase it then increase the value by .1 */
    dt.tankDrive(.7*(j.getLeftY()), .7*(j.getRightY()));

    
    /* .7 is the constant that controls the speed of the robot, if you want to increase it then increase the value by .1 */

   
   
  }
}