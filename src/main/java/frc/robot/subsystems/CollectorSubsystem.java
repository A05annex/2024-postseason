package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.a05annex.frc.subsystems.SparkNeo;

public class CollectorSubsystem extends SubsystemBase {

    private final SparkNeo motor = SparkNeo.factory(Constants.CAN_Devices.COLLECTOR_MOTOR);

    // Declare PID constants for smart motion control
    private final double smKp = 0.0, smKi = 0.0, smKiZone = 0.0, smKff = 0.000156, smMaxRPM = 3000.0,
            smMaxDeltaRPMSec = 3000.0, smMinRPM = 0.0, smError = 0.1;

    // Declare PID constants for position control
    private final double posKp = 0.0, posKi = 0.0, posKiZone = 0.0, posKff = 0.0;

    // Declare PID constants for speed (rpm) control
    private final double rpmKp = 0.0001, rpmKi = 0.0000002, rpmKiZone = 75.0, rpmKff = 0.000188;

    // Declare min and max soft limits and where the motor thinks it starts
    private final Double minPosition = null, maxPosition = null, startPosition = 0.0;

    private final double increment = 500.0;
    private double velocity = 0.0;

    private final static CollectorSubsystem INSTANCE = new CollectorSubsystem();
    public static CollectorSubsystem getInstance() {
        return INSTANCE;
    }

    private CollectorSubsystem() {
        motor.startConfig();
        motor.setCurrentLimit(SparkNeo.UseType.RPM_OCCASIONAL_STALL, SparkNeo.BreakerAmps.Amps40);
        motor.setSoftLimits(minPosition, maxPosition);
        motor.setDirection(SparkNeo.Direction.REVERSE);
        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        motor.setPositionPID(posKp, posKi, posKiZone, posKff);
        motor.setSmartMotion(smKp, smKi, smKiZone, smKff, smMaxRPM, smMaxDeltaRPMSec, smMinRPM, smError);
        motor.setRpmPID(rpmKp, rpmKi, rpmKiZone, rpmKff);
        motor.endConfig();
        motor.setEncoderPosition(startPosition);
    }

    public void setVelocity(double rpm) {
        motor.sparkMaxPID.setFF(calcFF(rpm), 0);
        motor.setTargetRPM(rpm);
    }

    public void intake() {
        setVelocity(5000.0);
    }

    public void stop() {
        motor.stopMotor();
    }

    public double getVelocity() {
        return motor.getEncoderVelocity();
    }

    public void incrementKff() {
        velocity += increment;
        setVelocity(velocity);
    }

    public void decrementKff() {
        velocity -= increment;
        setVelocity(velocity);
    }

    public double getReqVelocity() {
        return velocity;
    }

    private double calcFF(double rpm) {
        return 1.5357E-12 * Math.pow(rpm, 2.0) + -1.3564E-8 * Math.abs(rpm) + 0.0002211;
    }

}


