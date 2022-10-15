package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * .
 */
public class RestHeading extends CommandBase {
    private DrivetrainSubsystem m_drivetrainSubsystem;

    public RestHeading(DrivetrainSubsystem drivetrainSubsystem) {
        m_drivetrainSubsystem = drivetrainSubsystem;
    }
    
    @Override
    public void initialize() {
        m_drivetrainSubsystem.setModuleHomes();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}