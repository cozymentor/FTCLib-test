package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.command.CommandBase;
import java.util.function.DoubleSupplier;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class DefaultDriveCommand extends CommandBase {

    private final DriveSubsystem m_drive;
    private double m_leftX;
    private double m_leftY;
    private double m_rightX;
    private double  m_heading;

    public DefaultDriveCommand(DriveSubsystem drive, double leftX, double leftY, double rightX, double heading){
        m_drive = drive;
        m_leftX = leftX;
        m_leftY = leftY;
        m_rightX = rightX;
        m_heading = heading;
        addRequirements(m_drive);
    }

    public void initialize() {}

    @Override
    public void execute() {
        m_drive.drive(m_leftX, m_leftY, m_rightX, m_heading);
    }
}
