package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PathFollowingConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * .
 */
public class FollowPath extends CommandBase {
    private final PathPlannerTrajectory m_trajectory;
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final Timer m_timer;
    private final HolonomicDriveController m_controller;

    /**
     * .
     *
     * @param trajectory          .
     * @param drivetrainSubsystem .
     */
    public FollowPath(PathPlannerTrajectory trajectory, DrivetrainSubsystem drivetrainSubsystem) {
        m_trajectory = trajectory;
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_timer = new Timer();

        m_controller = new HolonomicDriveController(
                new PIDController(PathFollowingConstants.STRAFE_KP, PathFollowingConstants.STRAFE_KI,
                        PathFollowingConstants.STRAFE_KD),
                new PIDController(PathFollowingConstants.STRAFE_KP, PathFollowingConstants.STRAFE_KI,
                        PathFollowingConstants.STRAFE_KD),
                new ProfiledPIDController(PathFollowingConstants.ROTATION_KP, PathFollowingConstants.ROTATION_KI,
                        PathFollowingConstants.ROTATION_KD,
                        new Constraints(PathFollowingConstants.MAX_ROTATION_SPEED,
                                PathFollowingConstants.MAX_ROTATION_ACCELERATION)));

        addRequirements(m_drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
        m_drivetrainSubsystem.setPose(m_trajectory.getInitialPose());
    }

    @Override
    public void execute() {
        double curentTime = m_timer.get();
        PathPlannerState desiredState = (PathPlannerState) m_trajectory.sample(curentTime);

        ChassisSpeeds chassisSpeeds = m_controller.calculate(m_drivetrainSubsystem.getPose(), desiredState,
                desiredState.holonomicRotation);

        m_drivetrainSubsystem.setChassieSpeeds(chassisSpeeds);

    }
}
