package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Driver;
import frc.robot.Constants.Hardware;
import frc.robot.Constants.SwerveControlConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * Teleop drive command. Is Field Oriented.
 */
public class Drive extends CommandBase {
    private XboxController m_driver;
    private DrivetrainSubsystem m_drivetrainSubsystem;

    private SlewRateLimiter m_strafeX;
    private SlewRateLimiter m_strafeY;
    private boolean m_hold;
    private PIDController m_rotationPID;

    /**
     * Teleop drive command. Is Field Oriented.
     *
     * @param driver              The xBox controller that is the driver
     * @param drivetrainSubsystem The drivetrain subsystem
     */
    public Drive(XboxController driver, DrivetrainSubsystem drivetrainSubsystem) {
        m_driver = driver;
        m_drivetrainSubsystem = drivetrainSubsystem;

        m_strafeX = new SlewRateLimiter(5);
        m_strafeY = new SlewRateLimiter(5);

        addRequirements(m_drivetrainSubsystem);

        m_rotationPID = new PIDController(SwerveControlConstants.ROTATION_KP, SwerveControlConstants.ROTATION_KI,
                SwerveControlConstants.ROTATION_KD);
        m_rotationPID.enableContinuousInput(-Math.PI, Math.PI);

        // SmartDashboard.putNumber("Turn P", SwerveControlConstants.ROTATION_KP);

    }

    @Override
    public void execute() {
        // double newPTerm = SmartDashboard.getNumber("Turn P", 0);
        // if (newPTerm != m_rotationPID.getP()) {
        // m_rotationPID.setP(newPTerm);
        // }

        double strafeX = MathUtil.applyDeadband(m_strafeX.calculate(-m_driver.getLeftY()), Hardware.CONTROLLER_DEADBAND)
                * Driver.MAX_STRAFE_SPEED;
        double strafeY = MathUtil.applyDeadband(m_strafeY.calculate(-m_driver.getLeftX()), Hardware.CONTROLLER_DEADBAND)
                * Driver.MAX_STRAFE_SPEED;
        double rotate = MathUtil.applyDeadband(m_driver.getRightX(), Hardware.CONTROLLER_DEADBAND)
                * Driver.MAX_TURN_SPEED;

        // double rotation = m_drivetrainSubsystem.getGyroRotation().getDegrees();
        // if (rotate == 0) {
        // if (!m_hold) {
        // m_rotationPID.setSetpoint(rotation);
        // m_hold = true;
        // }

        // rotate = m_rotationPID.calculate(rotation);
        // }

        m_drivetrainSubsystem.drive(strafeX, strafeY, rotate,
                true);
    }

}
