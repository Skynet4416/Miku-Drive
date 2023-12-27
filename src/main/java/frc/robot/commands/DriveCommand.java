package frc.robot.commands;

import frc.robot.Constants.Drive;
import frc.robot.Constants.OI;
import frc.robot.subsystems.Drive.DriveSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class DriveCommand extends Command {
    private final DriveSubsystem mDriveSubsystem;
    private final DoubleSupplier mXDoubleSupplier;
    private final DoubleSupplier mYDoubleSupplier;
    private final DoubleSupplier mRotationDoubleSupplier;
    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public DriveCommand(DriveSubsystem driveSubsystem, DoubleSupplier xValue, DoubleSupplier yValue, DoubleSupplier rotationValue) {
        this.mDriveSubsystem = driveSubsystem;
        this.mXDoubleSupplier = xValue;
        this.mYDoubleSupplier = yValue;
        this.mRotationDoubleSupplier = rotationValue;

        addRequirements(driveSubsystem);
    }

    /**
     * Eliminates the drift from the joystick input
     */
    public double correctJoystickDrift(final double input) {
        return (Math.abs(input) > OI.XBOX_CONTROLLER_DRIFT) ? input : 0;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() 
    {
        mDriveSubsystem.setModules(
        correctJoystickDrift(mYDoubleSupplier.getAsDouble()) * Drive.Stats.MAX_VELOCITY_METERS_PER_SECOND,
        correctJoystickDrift(mXDoubleSupplier.getAsDouble()) * Drive.Stats.MAX_VELOCITY_METERS_PER_SECOND, 
        correctJoystickDrift(mRotationDoubleSupplier.getAsDouble()) * Drive.Stats.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
        );

    }



    @Override
    public void end(boolean interrupted) {
        mDriveSubsystem.setModules(0, 0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
