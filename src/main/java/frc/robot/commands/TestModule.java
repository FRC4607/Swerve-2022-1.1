package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * A command that added debug to Smart Dashboard for individually controlling a
 * swerve module.
 */
public class TestModule extends CommandBase {
    private DrivetrainSubsystem m_drivetrainSubsystem;

    /**
     * A command that added debug to Smart Dashboard for individually controlling a
     * swerve module..
     *
     * @param drivetrainSubsystem The drivetrain subsystem
     */
    public TestModule(DrivetrainSubsystem drivetrainSubsystem) {
        m_drivetrainSubsystem = drivetrainSubsystem;

        addRequirements(drivetrainSubsystem);

        SmartDashboard.putNumber("Turn Target", 0);
        SmartDashboard.putNumber("Drive Speed", 0);
        SmartDashboard.putNumber("Module Number", 0);

    }

    @Override
    public void execute() {

        SwerveModuleState[] moduleStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            moduleStates[i] = new SwerveModuleState();
        }

        moduleStates[(int) SmartDashboard.getNumber("Module Number", 0)] = new SwerveModuleState(
                SmartDashboard.getNumber("Drive Speed", 0), new Rotation2d(SmartDashboard.getNumber("Turn Target", 0)));

        m_drivetrainSubsystem.setModuleStates(moduleStates);
    }
}
