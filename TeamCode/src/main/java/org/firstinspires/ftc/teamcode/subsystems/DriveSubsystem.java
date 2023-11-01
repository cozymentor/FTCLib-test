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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

    private MotorEx leftFront, rightFront, leftBack, rightBack;

    private MecanumDrive m_drive;
    private MotorEx leftEncoder, rightEncoder, latEncoder;
    private GyroEx imu;

    private Telemetry telemetry;
    private HolonomicOdometry odometry;
    private MecanumDriveKinematics kinematics;

    public DriveSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.leftFront = new MotorEx(hardwareMap, "front_left", Motor.GoBILDA.RPM_223);
        this.rightFront = new MotorEx(hardwareMap, "front_right", Motor.GoBILDA.RPM_223);
        this.leftBack = new MotorEx(hardwareMap, "back_left", Motor.GoBILDA.RPM_223);
        this.rightBack = new MotorEx(hardwareMap, "back_right", Motor.GoBILDA.RPM_223);
        this.imu = new RevIMU(hardwareMap);
        this.imu.init();
        this.m_drive = new MecanumDrive(this.leftFront,this.rightFront,this.leftBack,this.rightBack);
        this.telemetry = telemetry;

    }

    @Override
    public void periodic() {
        this.telemetry.addLine("Drive Subsystem: online");
        this.telemetry.update();
    }

    public double getHeading() {
        return this.imu.getHeading();
    }
    public void drive(double strafeSpeed, double forwardSpeed, double turnSpeed, double gyroHeading) {
        this.m_drive.driveFieldCentric(strafeSpeed, forwardSpeed, turnSpeed, gyroHeading );
    }

}
