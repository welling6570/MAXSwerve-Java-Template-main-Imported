package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

public class Trajectories {
    private static List<Trajectory> results = new ArrayList<Trajectory>();
    private static Trajectory midTrajectory = new Trajectory();

    public static List<Trajectory> generateTrajectories(){
    TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,AutoConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(DriveConstants.kDriveKinematics);
    // Add kinematics to ensure max speed is actually obeyed


// An example trajectory to follow. All units in meters.
    midTrajectory = TrajectoryGenerator.generateTrajectory(
    // Start at the origin facing the +X direction
    new Pose2d(0, 0, new Rotation2d(3.1415)),
    // Pass through these two interior waypoints, making an 's' curve path
    List.of(new Translation2d(-.8, 0.1), new Translation2d(-1.6, -0.1)),
    //emptArr[],
    // End 3 meters straight ahead of where we started, facing forward
    new Pose2d(Units.inchesToMeters(-105), 0, new Rotation2d(0)),
    config);
        results.add(midTrajectory);
    return results;
    }
}
