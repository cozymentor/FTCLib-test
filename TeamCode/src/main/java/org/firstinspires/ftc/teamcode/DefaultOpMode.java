package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.external.samples.RobotTeleopPOV_Linear;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

@TeleOp(name = "DefaultOpMode")

public class DefaultOpMode extends CommandOpMode {
    private DriveSubsystem m_drive;
    private MotorEx fl, fr, bl,br;
    private RevIMU imu;
    @Override
    public void initialize() {
        fl = new MotorEx(hardwareMap, "front_left", Motor.GoBILDA.RPM_223);
        fr = new MotorEx(hardwareMap, "front_right", Motor.GoBILDA.RPM_223);
        bl = new MotorEx(hardwareMap, "back_left", Motor.GoBILDA.RPM_223);
        br = new MotorEx(hardwareMap, "back_right", Motor.GoBILDA.RPM_223);
        imu = new RevIMU(hardwareMap);
        m_drive = new DriveSubsystem(fl,fr,bl,br,imu);
        CommandScheduler.getInstance().reset();

        register();

        while (opModeInInit()) {
            telemetry.addLine("Robot Initialized.");
            telemetry.update();
        }
    }
    public void run() {
        while(opModeIsActive()){
            telemetry.addLine("Robot Running.");
            telemetry.update();
            super.run();}
    }
}
