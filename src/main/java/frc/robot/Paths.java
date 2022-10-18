package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

/**
 * The class that constanes all the preloaded Paths.
 */
public class Paths {
    public static final PathPlannerTrajectory FORWARD = PathPlanner.loadPath("Forward", 1.0, 0.5);
    public static final PathPlannerTrajectory OTHER_AUTO = PathPlanner.loadPath("Other Auto", 1.5, 2);
}