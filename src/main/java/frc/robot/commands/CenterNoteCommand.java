package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CollectorSubsystem;


public class CenterNoteCommand extends Command {
    private final CollectorSubsystem collectorSubsystem = CollectorSubsystem.getInstance();
    private enum STATUS {
        CENTER,
        REVERSE,
        FORWARD
    }

    private STATUS currentStatus;
    private int centerTimer = 0;
    private boolean isComplete;

    public CenterNoteCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(collectorSubsystem);
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        centerTimer = 0;
        isComplete = Constants.BEAMBREAK.get();
        if (!isComplete) {
            currentStatus = STATUS.CENTER;
            collectorSubsystem.setVelocity(2000.0);
        }
    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        if (isComplete) {
            return;
        }

        if (currentStatus == STATUS.CENTER) {
            centerTimer++;
            if (centerTimer > 50) {
                collectorSubsystem.setVelocity(-500.0);
                currentStatus = STATUS.REVERSE;
            }
        } else if (currentStatus == STATUS.REVERSE) {
            if (Constants.BEAMBREAK.get()) {
                collectorSubsystem.setVelocity(100.0);
                currentStatus = STATUS.FORWARD;
            }
        } else if (currentStatus == STATUS.FORWARD) {
            if (!Constants.BEAMBREAK.get()) {
                collectorSubsystem.stop();
                isComplete = true;
            }
        }
    }

    /**
     * <p>
     * Returns whether this command has finished. Once a command finishes -- indicated by
     * this method returning true -- the scheduler will call its {@link #end(boolean)} method.
     * </p><p>
     * Returning false will result in the command never ending automatically. It may still be
     * cancelled manually or interrupted by another command. Hard coding this command to always
     * return true will result in the command executing once and finishing immediately. It is
     * recommended to use * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand}
     * for such an operation.
     * </p>
     *
     * @return whether this command has finished.
     */
    @Override
    public boolean isFinished() {
        return false;
    }

    /**
     * The action to take when the command ends. Called when either the command
     * finishes normally -- that is it is called when {@link #isFinished()} returns
     * true -- or when  it is interrupted/canceled. This is where you may want to
     * wrap up loose ends, like shutting off a motor that was being used in the command.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {

    }
}
