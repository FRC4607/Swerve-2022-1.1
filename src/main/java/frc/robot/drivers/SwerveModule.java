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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import frc.robot.Constants.Hardware;
import frc.robot.Constants.SwerveModules;

/**
 * .
 */
public class SwerveModule implements AutoCloseable {
    private CANSparkMax m_driveMotor;
    private CANSparkMax m_turnMotor;
    private double m_home;
    private DutyCycleEncoder m_turnAbsoluteEncoder;
    private RelativeEncoder m_turnRelativeEncoder;
    private RelativeEncoder m_driveEncoder;
    private SparkMaxPIDController m_turnPIDControler;
    private SparkMaxPIDController m_drivePIDControler;
    private SwerveModuleState m_state;
    private DutyCycleEncoderSim m_turnAbsoluteEncoderSim;
    private double m_turnTarget;
    private double m_driveTarget;

    /**
     * An emcapushalasing of a swerve modual.
     *
     * @param driveMotorID The CAN ID of the drive motor
     * @param turnMotorID  The CAN ID of the turn motor
     * @param absEncoder   The roborio DIO port the absolut encoder is on
     * @param home         The reading of the Absolute encoder when the modual is at
     *                     the home rotation
     */
    public SwerveModule(String lable, int driveMotorID, int turnMotorID, int absEncoder, double home,
            boolean driverReversed) {
        m_turnAbsoluteEncoder = new DutyCycleEncoder(absEncoder);
        m_home = home;

        m_driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        m_turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);

        m_driveMotor.restoreFactoryDefaults();
        m_turnMotor.restoreFactoryDefaults();

        m_turnMotor.setIdleMode(IdleMode.kBrake);
        m_driveMotor.setIdleMode(IdleMode.kBrake);

        m_drivePIDControler = m_driveMotor.getPIDController();
        m_turnPIDControler = m_turnMotor.getPIDController();

        m_driveEncoder = m_driveMotor.getEncoder();
        m_turnRelativeEncoder = m_turnMotor.getEncoder();

        m_driveEncoder
                .setVelocityConversionFactor(Hardware.WHEEL_CIRCUMFERENCE_METERS / Hardware.DRIVE_GEER_RATIO / 60);
        m_driveEncoder.setPositionConversionFactor(Hardware.WHEEL_CIRCUMFERENCE_METERS / Hardware.DRIVE_GEER_RATIO);
        m_driveMotor.setInverted(driverReversed);
        m_turnRelativeEncoder.setPositionConversionFactor(2 * Math.PI / Hardware.TURN_GEER_RATIO);

        // m_turnPIDControler.setFeedbackDevice(m_turnRelativeEncoder);

        m_turnPIDControler.setP(SwerveModules.TURN_KP, SwerveModules.SLOT_ID);
        m_turnPIDControler.setI(SwerveModules.TURN_KI, SwerveModules.SLOT_ID);
        m_turnPIDControler.setD(SwerveModules.TURN_KD, SwerveModules.SLOT_ID);

        m_drivePIDControler.setP(SwerveModules.DRIVE_KP);
        m_drivePIDControler.setI(SwerveModules.DRIVE_KI);
        m_drivePIDControler.setD(SwerveModules.DRIVE_KD);
        m_drivePIDControler.setFF(SwerveModules.DRIVE_KF);

        ShuffleboardLayout layout = Shuffleboard.getTab("Drivetrain").getLayout(lable, "list");
        // layout.addNumber("Home", this::getHome);
        // layout.addNumber("Absolute Encoder", this::getAbsolutEncoder);
        // layout.addNumber("Turn Encoder", this::getTurnPos);
        // layout.addNumber("Turn Target", this::getTurnTaget);
        layout.addNumber("Wheel Velocity", () -> {
            return m_driveTarget - getWheelVeolocity();
        });
        layout.addNumber("Turn Error", () -> {
            return m_turnTarget - getTurnPos();
        });
    }

    public void homeEncoder() {
        m_turnRelativeEncoder.setPosition(getAbsolutEncoder() - m_home);
    }

    public void holdZero() {
        m_turnPIDControler.setReference(0, ControlType.kPosition);
    }

    /**
     * Returns the postion of the relative turn encoder in radian.
     */
    public double getTurnPos() {
        return m_turnRelativeEncoder.getPosition();
        // m_turnRelativeEncoder.set
    }

    /**
     * Returns the absolut position of the tune encder in radian.
     */
    public double getAbsolutEncoder() {
        return m_turnAbsoluteEncoder.getAbsolutePosition() * 2 * Math.PI;
    }

    public double getWheelVeolocity() {
        return m_driveEncoder.getVelocity();
    }

    public double getHome() {
        return m_home;
    }

    public double getTurnTaget() {
        return m_turnTarget;
    }

    /**
     * .
     */
    public void setModuleState(SwerveModuleState state) {
        m_state = SwerveModuleState.optimize(state, new Rotation2d(getTurnPos()));

        double relitiveEncoderValue = m_turnRelativeEncoder.getPosition();
        double target = m_state.angle.getRadians();

        target += Math.round((relitiveEncoderValue - target) / (2 * Math.PI)) * 2 * Math.PI;

        m_turnTarget = target;
        m_turnPIDControler.setReference(target, ControlType.kPosition);
        m_driveTarget = m_state.speedMetersPerSecond;
        m_drivePIDControler.setReference(m_state.speedMetersPerSecond, ControlType.kVelocity);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getWheelVeolocity(), new Rotation2d(getTurnPos()));
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
     * .
     */
    public DutyCycleEncoderSim getAbsolutEncoderSim() {
        if (m_turnAbsoluteEncoderSim == null) {
            m_turnAbsoluteEncoderSim = new DutyCycleEncoderSim(m_turnAbsoluteEncoder);
        }

        return m_turnAbsoluteEncoderSim;
    }

    /*
     * Questins:
     * Spark Max control loop frequencey
     * Pros/Cons of using external quad vs internal quad
     */
}