package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveWheelSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;

import org.firstinspires.ftc.teamcode.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

    private MotorGroup leftMotors, rightMotors;
    private HardwareMap hardwareMap;
    private Motor leftFront, rightFront, leftBack, rightBack;

    private MecanumDrive m_drive;
    private MotorEx leftEncoder, rightEncoder, latEncoder;
    private GyroEx imu;
    private HolonomicOdometry odometry;
    private MecanumDriveKinematics kinematics;

    public DriveSubsystem() {
        leftFront = new Motor(hardwareMap, "front_left");
        rightFront = new Motor(hardwareMap, "front_right");
        leftBack = new Motor(hardwareMap, "back_left");
        rightBack = new Motor(hardwareMap, "back_right");
        leftEncoder = new MotorEx(hardwareMap, "left odometer");
        rightEncoder = new MotorEx(hardwareMap, "right odometer");
        latEncoder = new MotorEx(hardwareMap, "lateral odometer");
        imu = new RevIMU(hardwareMap);
        imu.init();
        this.leftFront = leftFront;
        this.rightFront = rightFront;
        this.leftBack = leftBack;
        this.rightBack = rightBack;
        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder;
        this.latEncoder = latEncoder;
        this.imu = imu;

        odometry = new HolonomicOdometry(leftEncoder::getDistance,
                                        rightEncoder::getDistance,
                                        latEncoder::getDistance,
                                        DriveConstants.TRACK_WIDTH,
                                        DriveConstants.CENTER_WHEEL_OFFSET);

        kinematics = new MecanumDriveKinematics(new Translation2d(DriveConstants.leftFront_x,DriveConstants.leftFront_y),
                                                new Translation2d(DriveConstants.rightFront_x,DriveConstants.rightFront_y),
                                                new Translation2d(DriveConstants.leftBack_x,DriveConstants.leftBack_y),
                                                new Translation2d(DriveConstants.rightBack_x,DriveConstants.rightBack_y)
        );
    }

    @Override
    public void periodic() {
        odometry.updatePose();
    }

    public Pose2d getCurrentPose() {
        return odometry.getPose();
    }

    public void drive(double strafeSpeed, double forwardSpeed, double turnSpeed, double gyroHeading) {
        m_drive.driveFieldCentric(strafeSpeed, forwardSpeed, turnSpeed, gyroHeading );
    }

}
