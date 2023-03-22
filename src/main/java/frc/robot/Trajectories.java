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
        public static Trajectory bluemidTrajectory = new Trajectory();
        public static Trajectory redmidTrajectory = new Trajectory();
        public static Trajectory leftSideTrajectory = new Trajectory();
        public static Trajectory rightSideTrajectory = new Trajectory();
        public static Trajectory longTrajectory = new Trajectory();
        public static Trajectory gigaMidTrajectory = new Trajectory();
        public static Trajectory shortDockTrajectory = new Trajectory();
        public static Trajectory leaveChargeTrajectory = new Trajectory();
        public static Trajectory chargeEngageTrajectory = new Trajectory(); 
    public static void generateTrajectories(){
        //Config the Trajectory goodness!
        TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,AutoConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(DriveConstants.kDriveKinematics);


        // Basic Park-on-the-Charger Traj
        bluemidTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(Math.toRadians(180))),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(-.8, 0.1), new Translation2d(-1.6, -0.1)),
            //emptArr[],
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(Units.inchesToMeters(-118), 0, new Rotation2d(Math.toRadians(180))),
            config);
        //changed auto -120 to -118 after match 64
        //Changed rotation from 0 to 180 on 3/7/2023 6:59
        redmidTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(Math.toRadians(180))),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(-.8, 0.1), new Translation2d(-1.6, -0.1)),
            //emptArr[],
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(Units.inchesToMeters(-114), 0, new Rotation2d(Math.toRadians(180))),
            //Changed rotation from 0 to 180 on 3/7/2023 6:59
            //Changed distance from -121 to -118 changed from -118 to -116 before replay of match 47
            //changed distance from -116 to -114 after replay of match 47
            config);

        // Basic Park-on-the-Charger Traj
        leftSideTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(Math.toRadians(180))),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(-.8, 0.1), new Translation2d(-1.6, -0.1)),
            //emptArr[],
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(Units.inchesToMeters(-118), 0, new Rotation2d(Math.toRadians(180))),
            config);

        rightSideTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(Math.toRadians(180))),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(-.8, -0.1), new Translation2d(-2, 0.1)),
            //created right and left side trajectories. Set the polarity of the y variables. 3/10 9:54 am
            //emptArr[],
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(Units.inchesToMeters(-190), 0, new Rotation2d(Math.toRadians(180))),
            config);

        // Basic Park-on-the-Charger Traj
        longTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(3.1415)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(-.8, 0.1), new Translation2d(-1.6, -0.1)),
            //emptArr[],
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(Units.inchesToMeters(-190), 0, new Rotation2d(0)),
            config);

        leaveChargeTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(-118, 0, new Rotation2d(Math.toRadians(180))),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(-.8, 0.1), new Translation2d(-1.6, -0.1)),
            //emptArr[],
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(Units.inchesToMeters(-190), 0, new Rotation2d(Math.toRadians(180))),
            config);

        chargeEngageTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(-190, 0, new Rotation2d(Math.toRadians(180))),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(-.8, 0.1), new Translation2d(-1.6, -0.1)),
            //emptArr[],
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(Units.inchesToMeters(-118), 0, new Rotation2d(Math.toRadians(180))),
            config);
    } 

}
