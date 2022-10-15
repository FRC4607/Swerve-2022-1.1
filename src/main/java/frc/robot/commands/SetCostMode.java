package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * .
 */
public class SetCostMode extends CommandBase {
    private DrivetrainSubsystem m_drivetrainSubsystem;
    private boolean m_brakeMode;

    public SetCostMode(DrivetrainSubsystem drivetrainSubsystem, boolean brakeMode) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_brakeMode = brakeMode;
    }
    
    @Override
    public void initialize() {
        m_drivetrainSubsystem.setBrakeMode(m_brakeMode);
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
