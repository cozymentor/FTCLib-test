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

    private Motor leftFront, rightFront, leftBack, rightBack;

    private MecanumDrive m_drive;
    private MotorEx leftEncoder, rightEncoder, latEncoder;
    private GyroEx imu;
    private HolonomicOdometry odometry;
    private MecanumDriveKinematics kinematics;

    public DriveSubsystem(MotorEx frontLeft, MotorEx frontRight, MotorEx backLeft, MotorEx backRight, RevIMU imu) {
        this.leftFront = leftFront;
        this.rightFront = rightFront;
        this.leftBack = leftBack;
        this.rightBack = rightBack;
        this.imu = imu;
        this.imu.init();
        this.m_drive = new MecanumDrive(this.leftFront,this.rightFront,this.leftBack,this.rightBack);

    }

    @Override
    public void periodic() {
    }

    public double getHeading() {
        return this.imu.getHeading();
    }
    public void drive(double strafeSpeed, double forwardSpeed, double turnSpeed, double gyroHeading) {
        this.m_drive.driveFieldCentric(strafeSpeed, forwardSpeed, turnSpeed, gyroHeading );
    }

}
