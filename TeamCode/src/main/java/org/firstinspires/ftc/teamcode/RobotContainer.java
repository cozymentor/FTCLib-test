package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.command.CommandScheduler;
import org.firstinspires.ftc.teamcode.commands.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

public class RobotContainer {

    public final DriveSubsystem m_drive = new DriveSubsystem();

    private GamepadEx m_gamePad;
    public RobotContainer(HardwareMap hardwareMap, Gamepad gamePad) {
        m_drive.init(hardwareMap);
        m_gamePad = new GamepadEx(gamePad);
        configureButtonBindings();
        m_drive.setDefaultCommand(new DefaultDriveCommand(m_drive, m_gamePad.getLeftX(), m_gamePad.getLeftY(), m_gamePad.getRightX(), m_drive.getHeading()));
    }


    private void configureButtonBindings(){
        m_gamePad.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new InstantCommand());
    }
}
