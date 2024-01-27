package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.Subsystems.Tank;

public class DriveToDistance extends PIDCommand{
    Tank m_tank;
    
    public DriveToDistance(double distMeters, Tank m_tank)
    {
        super(
            new PIDController(1.15, 0, 0),
            m_tank::getDistanceMeters, ////what is being measured
            distMeters, ////where you want to be
            output -> m_tank.drive(output, 0), ////the output to get to distMeters
            m_tank
        );
        
        ////sets tolerable error
        getController().setTolerance(Units.inchesToMeters(5)); 

        this.m_tank = m_tank;
    }
    
    //when
    @Override
    public void initialize()
    {
        m_tank.resetEncoders();
    }

    @Override
    public boolean isFinished()
    {
        return getController().atSetpoint();    //  This command will terminate once the desired distance has been reached.
    }
    
    @Override
    public void end(boolean interrupted) {
        m_tank.drive(0, 0);
        m_tank.resetEncoders();
    }
}