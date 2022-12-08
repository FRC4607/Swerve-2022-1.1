package frc.robot.drivers;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Hardware;
import frc.robot.Constants.SwerveModuleConstants;

/**
 * A class that encapsulates a swerve module.
 */
public class SwerveModule implements AutoCloseable {
    private CANSparkMax m_driveMotor;
    private CANSparkMax m_turnMotor;
    private double m_home;
    private DutyCycleEncoder m_turnAbsoluteEncoder;
    private RelativeEncoder m_turnRelativeEncoder;
    private RelativeEncoder m_driveEncoder;
    private SparkMaxPIDController m_turnPIDController;
    private SparkMaxPIDController m_drivePIDController;
    private SwerveModuleState m_state;
    private DutyCycleEncoderSim m_turnAbsoluteEncoderSim;
    private double m_turnTarget;
    private double m_driveTarget;
    private String m_label;

    /**
     * An encapsulates of a swerve module.
     *
     * @param driveMotorID The CAN ID of the drive motor
     * @param turnMotorID  The CAN ID of the turn motor
     * @param absEncoder   The roborio DIO port the absolute encoder is on
     * @param home         The reading of the Absolute encoder when the module is at
     *                     the home rotation
     */
    public SwerveModule(String label, int driveMotorID, int turnMotorID, int absEncoder,
            double home, boolean driverReversed) {
        m_label = label;

        m_home = Preferences.getDouble(m_label + ":home", -100);
        if (m_home == -100) {
            Preferences.setDouble(m_label + ":home", home);
            m_home = home;
        }

        m_turnAbsoluteEncoder = new DutyCycleEncoder(absEncoder);
        m_driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        m_turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);

        m_driveMotor.restoreFactoryDefaults();
        m_turnMotor.restoreFactoryDefaults();

        m_turnMotor.setIdleMode(IdleMode.kBrake);
        m_driveMotor.setIdleMode(IdleMode.kBrake);

        m_drivePIDController = m_driveMotor.getPIDController();
        m_turnPIDController = m_turnMotor.getPIDController();

        m_driveEncoder = m_driveMotor.getEncoder();
        m_turnRelativeEncoder = m_turnMotor.getEncoder();

        m_driveEncoder
                .setVelocityConversionFactor(Hardware.WHEEL_CIRCUMFERENCE_METERS / Hardware.DRIVE_GEAR_RATIO / 60);
        m_driveEncoder.setPositionConversionFactor(Hardware.WHEEL_CIRCUMFERENCE_METERS / Hardware.DRIVE_GEAR_RATIO);
        m_driveMotor.setInverted(driverReversed);
        m_turnRelativeEncoder.setPositionConversionFactor(2 * Math.PI / Hardware.TURN_GEAR_RATIO);

        // m_turnPIDController.setFeedbackDevice(m_turnRelativeEncoder);

        m_turnPIDController.setP(SwerveModuleConstants.TURN_KP, SwerveModuleConstants.SLOT_ID);
        m_turnPIDController.setI(SwerveModuleConstants.TURN_KI, SwerveModuleConstants.SLOT_ID);
        m_turnPIDController.setD(SwerveModuleConstants.TURN_KD, SwerveModuleConstants.SLOT_ID);

        m_drivePIDController.setP(SwerveModuleConstants.DRIVE_KP);
        m_drivePIDController.setI(SwerveModuleConstants.DRIVE_KI);
        m_drivePIDController.setD(SwerveModuleConstants.DRIVE_KD);
        m_drivePIDController.setFF(SwerveModuleConstants.DRIVE_KF);

        ShuffleboardLayout layout = Shuffleboard.getTab("Drivetrain").getLayout(label, "list");
        layout.addNumber("Home", this::getHome);
        layout.addNumber("Absolute Encoder", this::getAbsoluteEncoder);
        // layout.addNumber("Turn Encoder", this::getTurnPos);
        // layout.addNumber("Turn Target", this::getTurnTarget);
        // layout.addNumber("Wheel Velocity", () -> {
        // return m_driveTarget - getWheelVelocity();
        // });
        // layout.addNumber("Turn Error", () -> {
        // return m_turnTarget - getTurnPos();
        // });
    }

    /**
     * Sets the current position of the module as home.
     */
    public void setCurrentHome() {
        m_home = getAbsoluteEncoder();
        Preferences.setDouble(m_label + ":home", m_home);

        homeEncoder();
    }

    public void homeEncoder() {
        m_turnRelativeEncoder.setPosition(getAbsoluteEncoder() - m_home);
    }

    public void holdZero() {
        m_turnPIDController.setReference(0, ControlType.kPosition);
    }

    /**
     * Returns the position of the relative turn encoder in radian.
     */
    public double getTurnPos() {
        return m_turnRelativeEncoder.getPosition();
        // m_turnRelativeEncoder.set
    }

    /**
     * Returns the absolute position of the turn encoder in radian.
     */
    public double getAbsoluteEncoder() {
        return m_turnAbsoluteEncoder.getAbsolutePosition() * 2 * Math.PI;
    }

    public double getWheelVelocity() {
        return m_driveEncoder.getVelocity();
    }

    public double getHome() {
        return m_home;
    }

    public double getTurnTarget() {
        return m_turnTarget;
    }

    /**
     * Sets the target module state.
     */
    public void setModuleState(SwerveModuleState state) {
        m_state = SwerveModuleState.optimize(state, new Rotation2d(getTurnPos()));

        double relativeEncoderValue = m_turnRelativeEncoder.getPosition();
        double target = m_state.angle.getRadians();

        target += Math.round((relativeEncoderValue - target) / (2 * Math.PI)) * 2 * Math.PI;

        if (Math.abs(m_state.speedMetersPerSecond) > 0.1) {
            m_turnTarget = target;
            m_turnPIDController.setReference(target, ControlType.kPosition);
        }
        m_driveTarget = m_state.speedMetersPerSecond;
        m_drivePIDController.setReference(m_state.speedMetersPerSecond, ControlType.kVelocity);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getWheelVelocity(), new Rotation2d(getTurnPos()));
    }

    public void update() {
    }

    /*
     * ==========================\\
     * || Methods for unit testing ||
     * \\==========================
     */

    public String helloWorld(String string) {
        System.out.println(string);
        return string;
    }

    @Override
    public void close() throws Exception {
        m_driveMotor.close();
        m_turnMotor.close();
        m_turnAbsoluteEncoder.close();
    }

    /**
     * Return the encoder sim for testing.
     */
    public DutyCycleEncoderSim getAbsoluteEncoderSim() {
        if (m_turnAbsoluteEncoderSim == null) {
            m_turnAbsoluteEncoderSim = new DutyCycleEncoderSim(m_turnAbsoluteEncoder);
        }

        return m_turnAbsoluteEncoderSim;
    }

    public void setBrakeMode(boolean brakeMode) {
        m_driveMotor.setIdleMode(brakeMode ? IdleMode.kBrake : IdleMode.kCoast);
        m_turnMotor.setIdleMode(brakeMode ? IdleMode.kBrake : IdleMode.kCoast);
    }

    /*
     * Questions:
     * Spark Max control loop frequency
     * Pros/Cons of using external quad vs internal quad
     */
}