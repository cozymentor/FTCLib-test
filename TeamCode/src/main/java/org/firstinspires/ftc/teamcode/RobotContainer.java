package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.command.CommandScheduler;
import org.firstinspires.ftc.teamcode.commands.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

public class RobotContainer {
    private final DriveSubsystem m_drive = new DriveSubsystem();
    GamepadEx m_gamePad;
    public RobotContainer() {
        configureButtonBindings();
        m_drive.setDefaultCommand(new DefaultDriveCommand(m_drive, m_gamePad.getLeftX(), m_gamePad.getLeftY(), m_gamePad.getRightX(), m_drive.getHeading()));
    }

    private void configureButtonBindings(){
        m_gamePad.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new InstantCommand());


    }
}
