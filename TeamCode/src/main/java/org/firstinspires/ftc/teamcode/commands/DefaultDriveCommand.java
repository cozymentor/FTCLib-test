package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import java.util.function.DoubleSupplier;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class DefaultDriveCommand extends CommandBase {

    private final DriveSubsystem m_drive;
    private final DoubleSupplier m_leftX;
    private final DoubleSupplier m_leftY;
    private final DoubleSupplier m_rightX;
    private final DoubleSupplier m_heading;

    public DefaultDriveCommand(DriveSubsystem drive, DoubleSupplier leftX, DoubleSupplier leftY, DoubleSupplier rightX, DoubleSupplier heading){
        m_drive = drive;
        m_leftX = leftX;
        m_leftY = leftY;
        m_rightX = rightX;
        m_heading = heading;
        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        m_drive.drive(m_leftX.getAsDouble(), m_leftY.getAsDouble(), m_rightX.getAsDouble(), m_heading.getAsDouble());
    }
}
