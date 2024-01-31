package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase{
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

  /**
   * How many amps the arm motor can use.
   */
  static final int ARM_CURRENT_LIMIT_A = 20;


  /**
   * Percent output to run the arm up/down at
   */
  static final double ARM_OUTPUT_POWER = 0.35;


   /**
   * Time to extend or retract arm in auto
   */
  static final double ARM_EXTEND_TIME_S = 2.0;

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
}
