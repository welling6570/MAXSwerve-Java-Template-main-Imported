package frc.robot;

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
        public static Trajectory midTrajectory = new Trajectory();
        public static Trajectory shortTrajectory = new Trajectory();
        public static Trajectory longTrajectory = new Trajectory();
        public static Trajectory gigaMidTrajectory = new Trajectory();
    
public static void generateTrajectories(){
    //Config the Trajectory goodness!
    TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,AutoConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(DriveConstants.kDriveKinematics);


        // Basic Park-on-the-Charger Traj
        midTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(3.1415)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(-.8, 0.1), new Translation2d(-1.6, -0.1)),
            //emptArr[],
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(Units.inchesToMeters(-105), 0, new Rotation2d(0)),
            config);

        // Basic Park-on-the-Charger Traj
        shortTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(3.1415)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(-.8, 0.1), new Translation2d(-1.6, -0.1)),
            //emptArr[],
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(Units.inchesToMeters(-105), 0, new Rotation2d(0)),
            config);

        // Basic Park-on-the-Charger Traj
        longTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(3.1415)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(-.8, 0.1), new Translation2d(-1.6, -0.1)),
            //emptArr[],
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(Units.inchesToMeters(-105), 0, new Rotation2d(0)),
            config);

        // Basic Park-on-the-Charger Traj
        gigaMidTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(3.1415)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(-.8, 0.1), new Translation2d(-1.6, -0.1)),
            //emptArr[],
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(Units.inchesToMeters(-105), 0, new Rotation2d(0)),
            config);
    }
}
