package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Driver;
import frc.robot.Constants.Hardware;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * Teleop drive command. Is Field Orented. 
 */
public class Drive extends CommandBase {
    private XboxController m_driver;
    private DrivetrainSubsystem m_drivetrainSubsystem;

    /**
     * Teleop drive command. Is Field Orented.
     *
     * @param driver The xBox controler that is the driver
     * @param drivetrainSubsystem The drivetrain subsystem
     */
    public Drive(XboxController driver, DrivetrainSubsystem drivetrainSubsystem) {
        m_driver = driver;
        m_drivetrainSubsystem = drivetrainSubsystem;

        addRequirements(m_drivetrainSubsystem);
    }

    @Override
    public void execute() {
        m_drivetrainSubsystem.drive(
                MathUtil.applyDeadband(-m_driver.getLeftY(), Hardware.CONTROLER_DEADBAND) * Driver.MAX_STRAFE_SPEED,
                MathUtil.applyDeadband(-m_driver.getLeftX(), Hardware.CONTROLER_DEADBAND) * Driver.MAX_STRAFE_SPEED,
                MathUtil.applyDeadband(m_driver.getRightX(), Hardware.CONTROLER_DEADBAND) * Driver.MAX_TURN_SPEED,
                true);
    }

}
