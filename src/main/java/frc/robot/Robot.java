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


import edu.wpi.first.math.controller.PIDController;


import frc.robot.lib.DataLog;

public class Robot extends TimedRobot {
  private RobotContainer m_robotContainer;
  private Command m_autonomousCommand;
  private String m_autoSelected;

  /**
   * This method is run once when the robot is first started up.
   */
  @Override
  public void robotInit() {
    DataLog.start();
    m_robotContainer = new RobotContainer();    
  }



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

   
   
  }
}
