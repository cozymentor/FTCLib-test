package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.external.samples.RobotTeleopPOV_Linear;

@TeleOp(name = "DefaultOpMode")

public class DefaultOpMode extends CommandOpMode {
    private RobotContainer m_robotContainer;

    @Override
    public void initialize() {
        m_robotContainer = new RobotContainer();
        CommandScheduler.getInstance().reset();
    }
    public void run() {
        CommandScheduler.getInstance().run();
    }
}
