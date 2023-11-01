package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
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
import org.firstinspires.ftc.teamcode.commands.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

@TeleOp(name = "DefaultOpMode")

public class DefaultOpMode extends CommandOpMode {
    private DriveSubsystem m_drive;
    private MotorEx fl, fr, bl,br;
    private GamepadEx driver_op;
    private RevIMU imu;

    private DefaultDriveCommand driveCommand;
    @Override
    public void initialize() {
        driver_op = new GamepadEx(gamepad1); //Convert default gamepad object to GamepadEx object
        //Initialize Subsystems
        m_drive = new DriveSubsystem(hardwareMap, telemetry);
        register(m_drive);  //Register Subsystems

        //Initialize Commands
        driveCommand = new DefaultDriveCommand(m_drive, driver_op::getLeftX, driver_op::getLeftY, driver_op::getRightX, m_drive::getHeading);
        //Set Subsystem Default Commands
        m_drive.setDefaultCommand(driveCommand);
        configureButtonBindings(); //Runs the configureButtonBindings() method below,
        super.reset();


        //Runs while robot in Init stage
        while (opModeInInit()) {
            telemetry.addLine("Robot Initialized.");
            telemetry.update();
        }
    }
    private void configureButtonBindings(){
        //put all button bindings here
        driver_op.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new InstantCommand());
    }
    
}
