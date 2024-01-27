package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Tank;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.auto.DriveToDistance;
import frc.robot.commands.auto.TurnToAngle;
import frc.robot.commands.drive.DriveWithJoysticks;

public class RobotContainer {
    private final SendableChooser<Command> m_chooser = new SendableChooser<Command>();
    private Tank m_tank = new Tank();

    private final CommandXboxController m_driverController = new CommandXboxController(Constants.Controllers.kDriverControllerPort);
    private final CommandXboxController m_operatorController = new CommandXboxController(Constants.Controllers.kOperatorControllerPort);

    public RobotContainer() {
        setupDrive(); 
        configureButtonBindings();
        //Creating a dropdown for autonomous commands to choose from
        addCommandDropdown();
        SmartDashboard.putData("Auto choices", m_chooser);
    }

    public Command getAutonomousCommand(){
      return m_chooser.getSelected();
    }

    private void setupDrive() {
        m_tank.setDefaultCommand(new RunCommand(() -> m_tank.drive(m_driverController.getLeftY(), m_driverController.getRightX()) ));
    }


    private void configureButtonBindings() {
    //DRIVER
    m_driverController.a().onTrue(new InstantCommand(() -> m_tank.resetEncoders(), m_tank));

   

    //m_operatorController.x().onTrue(new InstantCommand(() -> m_arm.getExtensionEncoder().setPosition(0), m_arm));

    //TODO: Fix Arm Angle Offsets in Arm.java first before uncommenting
    // m_operatorController.a().onTrue(new MoveToGoal(m_arm, Row.BOTTOM))
    // .or(m_operatorController.b().onTrue(new MoveToGoal(m_arm, Row.MIDDLE)))
    // .or(m_operatorController.y().onTrue(new MoveToGoal(m_arm, Row.TOP)));
    
    //slow mode for right bumper, medium slow for left bumper
    m_driverController.rightBumper().onTrue(new InstantCommand(() -> m_tank.evilMode(), m_tank))
    .or(m_driverController.leftBumper().onTrue(new InstantCommand(() -> m_tank.goodMode(), m_tank)))
    .onFalse(new InstantCommand(() -> m_tank.fastMode(), m_tank));
  }

  private void addCommandDropdown(){
    m_chooser.setDefaultOption("Drive to Distance", new DriveToDistance(5, m_tank));
  }


    
}