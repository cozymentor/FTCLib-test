package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

public class RobotContainer {
    private final DriveSubsystem m_drive = new DriveSubsystem();
    GamepadEx m_gamePad;
    public RobotContainer() {
        configureButtonBindings();
    }

    private void configureButtonBindings(){
        m_gamePad.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(() -> schedule(new TeleOpAutoGrabCommand(intake, () -> gamepad2.right_trigger > 0.5)));
        m_gamePad.getGamepadButton(Button.A)

    }
}
