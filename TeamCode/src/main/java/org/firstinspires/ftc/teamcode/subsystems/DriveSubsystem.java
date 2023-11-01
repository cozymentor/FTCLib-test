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
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Constants.HardwareMapConstants;

public class DriveSubsystem extends SubsystemBase {
    private final MotorEx leftFront, rightFront, leftBack, rightBack;
    private final MecanumDrive m_drive;
    private final GyroEx imu;
    private final Telemetry telemetry;

    public DriveSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.leftFront = new MotorEx(hardwareMap, HardwareMapConstants.leftFront, Motor.GoBILDA.RPM_223);
        this.rightFront = new MotorEx(hardwareMap, HardwareMapConstants.rightFront, Motor.GoBILDA.RPM_223);
        this.leftBack = new MotorEx(hardwareMap, HardwareMapConstants.leftBack, Motor.GoBILDA.RPM_223);
        this.rightBack = new MotorEx(hardwareMap, HardwareMapConstants.rightBack, Motor.GoBILDA.RPM_223);
        this.imu = new RevIMU(hardwareMap);

        this.m_drive = new MecanumDrive(this.leftFront,this.rightFront,this.leftBack,this.rightBack);
        this.telemetry = telemetry;
        //Initialization section
        this.imu.init();
        this.leftFront.setInverted(true);
        this.rightFront.setInverted(true);
        this.leftBack.setInverted(true);
        this.rightBack.setInverted(true);
        System.out.println("Heck");
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
