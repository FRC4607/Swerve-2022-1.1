package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Hardware;
import frc.robot.Constants.SwerveModuleConstatntes;
import frc.robot.drivers.SwerveModule;

/**
 * The Drivetrain Subsystem.
 */
public class DrivetrainSubsystem extends SubsystemBase {

    private SwerveModule[] m_swerveModules;
    private Timer m_homingTimer;

    private Pigeon2 m_pideon;
    private SwerveDriveKinematics m_kinematics;
    private SwerveDriveOdometry m_odometry;

    /**
     * .
     */
    public DrivetrainSubsystem() {
        m_swerveModules = new SwerveModule[SwerveModuleConstatntes.LABLES.length];

        if (SwerveModuleConstatntes.LABLES.length != SwerveModuleConstatntes.DRIVE_CAN_IDS.length
                || SwerveModuleConstatntes.LABLES.length != SwerveModuleConstatntes.TURN_CAN_IDS.length
                || SwerveModuleConstatntes.LABLES.length != SwerveModuleConstatntes.ABS_ENCODER_DIO_PORT.length
                || SwerveModuleConstatntes.LABLES.length != SwerveModuleConstatntes.HOMES_RAD.length) {
            DriverStation.reportError("The number of values in the module config is not equal", true);
        }
        for (int i = 0; i < SwerveModuleConstatntes.LABLES.length; i++) {
            m_swerveModules[i] = new SwerveModule(
                    SwerveModuleConstatntes.LABLES[i],
                    SwerveModuleConstatntes.DRIVE_CAN_IDS[i],
                    SwerveModuleConstatntes.TURN_CAN_IDS[i],
                    SwerveModuleConstatntes.ABS_ENCODER_DIO_PORT[i],
                    SwerveModuleConstatntes.HOMES_RAD[i],
                    SwerveModuleConstatntes.DRIVE_ENCODER_REVERSED[i]);
        }
        m_homingTimer = new Timer();
        m_homingTimer.start();

        m_pideon = new Pigeon2(Hardware.PIDEON_CAN_ID);
        m_pideon.configFactoryDefault();
        m_pideon.configMountPoseYaw(90);
        m_pideon.setYaw(0);

        m_kinematics = new SwerveDriveKinematics(SwerveModuleConstatntes.POSITIONS);
        m_odometry = new SwerveDriveOdometry(m_kinematics, getGyroRotation());
    }

    public Rotation2d getGyroRotation() {
        return Rotation2d.fromDegrees(m_pideon.getYaw());
    }

    @Override
    public void periodic() {

        if (m_homingTimer.hasElapsed(2)) {
            for (int i = 0; i < SwerveModuleConstatntes.LABLES.length; i++) {
                m_swerveModules[i].homeEncoder();
            }
            m_homingTimer.stop();
            m_homingTimer.reset();
        }

        SwerveModuleState[] moduleStates = new SwerveModuleState[m_swerveModules.length];
        for (int i = 0; i < m_swerveModules.length; i++) {
            moduleStates[i] = m_swerveModules[i].getState();
        }
        m_odometry.update(getGyroRotation(), moduleStates);

        SmartDashboard.putNumber("Gyro Yaw", m_pideon.getYaw());
        SmartDashboard.putNumber("Odometry Yaw", m_odometry.getPoseMeters().getRotation().getDegrees());
    }

    /**
     * .
     */
    public void setModuleStates(SwerveModuleState[] swerveModuleStates) {
        if (swerveModuleStates.length != SwerveModuleConstatntes.LABLES.length) {
            DriverStation.reportError(
                    "The number of module states provided is not the same as the number of modules", true);
            return;
        }
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Hardware.MAX_SPEED_METER_PER_SECOND);
        for (int i = 0; i < swerveModuleStates.length; i++) {
            m_swerveModules[i].setModuleState(swerveModuleStates[i]);
        }
    }

    /**
     * Driver input Method.
     *
     * @param strafeX        Meters per sec
     * @param strafeY        Meters per sec
     * @param rotate         rad per sec
     * @param fieldOrentated whether of not to be relitive to the field or the robot
     */
    public void drive(double strafeX, double strafeY, double rotate, boolean fieldOrentated) {
        ChassisSpeeds chassisSpeed;

        if (fieldOrentated) {
            chassisSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(strafeX, strafeY, rotate,
                    m_odometry.getPoseMeters().getRotation());
        } else {
            chassisSpeed = new ChassisSpeeds(strafeX, strafeY, rotate);
        }
        SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(chassisSpeed);

        setModuleStates(moduleStates);

    }

    /**
     * .
     */
    public void setModuleHomes() {
        for (int i = 0; i < m_swerveModules.length; i++) {
            m_swerveModules[i].setCurentHome();
        }
    }

    /**
     * .
     *
     * @param brakeMode Weather or not be in brake mode.
     */
    public void setBrakeMode(boolean brakeMode) {
        for (int i = 0; i < m_swerveModules.length; i++) {
            m_swerveModules[i].setBrakeMode(brakeMode);
        }
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public void resetHeading() {
        m_odometry.resetPosition(getPose(), new Rotation2d());
    }
}
