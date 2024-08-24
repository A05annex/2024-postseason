package frc.robot.subsystems;


import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.a05annex.frc.subsystems.SparkNeo;
import org.a05annex.util.Utl;

public class ShooterSubsystem extends SubsystemBase {

    private double requestedRPM = 0;

    private final SparkNeo leftMotor = SparkNeo.factory(Constants.CAN_Devices.LEFT_SHOOTER_MOTOR);
    private final SparkNeo rightMotor = SparkNeo.factory(Constants.CAN_Devices.RIGHT_SHOOTER_MOTOR);

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

    private final static ShooterSubsystem INSTANCE = new ShooterSubsystem();
    public static ShooterSubsystem getInstance() {
        return INSTANCE;
    }

    private ShooterSubsystem() {
        leftMotor.startConfig();
        leftMotor.setCurrentLimit(SparkNeo.UseType.FREE_SPINNING, SparkNeo.BreakerAmps.Amps40);
        leftMotor.setSoftLimits(minPosition, maxPosition);
        leftMotor.setDirection(SparkNeo.Direction.REVERSE);
        leftMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        leftMotor.setPositionPID(posKp, posKi, posKiZone, posKff);
        leftMotor.setSmartMotion(smKp, smKi, smKiZone, smKff, smMaxRPM, smMaxDeltaRPMSec, smMinRPM, smError);
        leftMotor.setRpmPID(rpmKp, rpmKi, rpmKiZone, rpmKff);
        leftMotor.endConfig();
        leftMotor.setEncoderPosition(startPosition);

        rightMotor.startConfig();
        rightMotor.setCurrentLimit(SparkNeo.UseType.FREE_SPINNING, SparkNeo.BreakerAmps.Amps40);
        rightMotor.setSoftLimits(minPosition, maxPosition);
        rightMotor.setDirection(SparkNeo.Direction.DEFAULT);
        rightMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        rightMotor.setPositionPID(posKp, posKi, posKiZone, posKff);
        rightMotor.setSmartMotion(smKp, smKi, smKiZone, smKff, smMaxRPM, smMaxDeltaRPMSec, smMinRPM, smError);
        rightMotor.setRpmPID(rpmKp, rpmKi, rpmKiZone, rpmKff);
        rightMotor.endConfig();
        rightMotor.setEncoderPosition(startPosition);

        requestedRPM = 0;
    }


    public void setVelocity(double rpm) {
        //leftMotor.sparkMaxPID.setFF(calcFF(rpm), 0);
        leftMotor.setTargetRPM(rpm);
        rightMotor.setTargetRPM(rpm);
        requestedRPM = rpm;
    }

    public void fullPower() {
        leftMotor.sparkMaxPID.setReference(12.0, CANSparkBase.ControlType.kVoltage);
        rightMotor.sparkMaxPID.setReference(12.0, CANSparkBase.ControlType.kVoltage);
    }

    public boolean isAtRPM(double rpm) {
        return Utl.inTolerance(getVelocity(), rpm, 100.0);
    }

    public boolean isAtRPM() {
        return isAtRPM(requestedRPM);
    }

    public void stop() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
        requestedRPM = 0;
    }

    public double getVelocity() {
        return leftMotor.getEncoderVelocity();
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


