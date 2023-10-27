package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RamseteCommand;
import com.arcrobotics.ftclib.controller.wpilibcontroller.RamseteController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveKinematics;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.Arrays;

@Autonomous
public class AutonomousOpMode extends CommandOpMode {

    private Motor frontLeft, frontRight, backLeft, backRight;
    private RevIMU imu;
    private RobotContainer m_robotContainer;
    private MotorEx leftEncoder, rightEncoder,latEncoder;
    private RamseteCommand ramseteFollower;


    @Override
    public void initialize() {


        m_robotContainer = new RobotContainer();
        DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(DriveConstants.TRACK_WIDTH);
        TrajectoryConfig config = new TrajectoryConfig(DriveConstants.MAX_VELOCITY, DriveConstants.MAX_ACCELERATION)
                .setKinematics(kinematics);

        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                Arrays.asList(new Translation2d(1, 1), new Translation2d(2, -1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, new Rotation2d(0)),
                // Pass config
                config
        );

        ramseteFollower = new RamseteCommand(
                exampleTrajectory,
                driveSubsystem::getCurrentPose,
                new RamseteController(DriveConstants.B, DriveConstants.ZETA),
                kinematics,
                driveSubsystem::drive
        );

        schedule(ramseteFollower);
    }

}
