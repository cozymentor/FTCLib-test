package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import java.util.function.DoubleSupplier;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class DefaultDriveCommand extends CommandBase {

    private final DriveSubsystem m_drive;

    public DefaultDriveCommand(DriveSubsystem drive, DoubleSupplier leftX, DoubleSupplier leftY, DoubleSupplier rightX, DoubleSupplier heading){
        m_drive = drive;
        drive.drive(leftX.getAsDouble(), leftY.getAsDouble(), rightX.getAsDouble(), heading.getAsDouble());
    }
}
