package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.Subsystems.Tank;

public class TurnToAngle extends PIDCommand{
    Tank m_tank;
    
    public TurnToAngle(double angle, Tank m_tank)
    {
        super(
            new PIDController(0.015, 0, 0),
            m_tank::getHeading,
            angle + m_tank.getHeading(),
            output -> m_swerve.drive(0, 0, output, false),
            m_tank
        );

        getController().setTolerance(3);

        this.m_swerve = m_swerve;
    }

    @Override
    public boolean isFinished()
    {
        return getController().atSetpoint();    //  This command will terminate once the desired distance has been reached.
    }
    
    @Override
    public void end(boolean interrupted) {
        m_swerve.drive(0, 0, 0, false);
    }
}