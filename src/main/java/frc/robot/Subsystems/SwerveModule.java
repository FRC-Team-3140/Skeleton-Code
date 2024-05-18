package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Libs.AbsoluteEncoder;

public class SwerveModule extends SubsystemBase {

    // Zero : 0.697578
    // One : 0.701239
    // Two: 0.467096
    // Three : 0.207867
    public String moduleID;
    public int pwmID;
    public int driveMotorID;
    public int turnMotorID;
    public double baseAngle;
    public CANSparkMax turnMotor;
    public CANSparkMax driveMotor;
    public PIDController turnPID;
    public ProfiledPIDController drivePID;
    public AbsoluteEncoder turnEncoder;
    public RelativeEncoder driveEncoder;

    public double botMass = 24.4;

    public double P = .01;

    public double driveSetpointTolerance = .5;
    public double turnSetpointTolerance;
    public double turnVelocityTolerance;

    private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.084706 * .712, 2.4433 * .712,
            0.10133 * .712);
    // realised the feedforward was off by a factor of .712, corrected it
    private TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(Constants.maxChassisSpeed,
            Constants.maxAcceleration);

   
    public SwerveModule(String moduleID, int analogID, int driveMotorID, int turnMotorID, double baseAngle) {
        this.moduleID = moduleID;
        this.baseAngle = baseAngle;
        this.turnMotorID = turnMotorID;
        this.driveMotorID = driveMotorID;

        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        driveMotor.restoreFactoryDefaults();
        driveMotor.setIdleMode(IdleMode.kBrake);
        driveMotor.setInverted(false);
        driveMotor.setSmartCurrentLimit(40);
        driveMotor.burnFlash();

        turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);
        turnMotor.restoreFactoryDefaults();
        turnMotor.setIdleMode(IdleMode.kBrake);
        turnMotor.setInverted(false);
        turnMotor.setSmartCurrentLimit(30);
        turnMotor.burnFlash();

        turnEncoder = new AbsoluteEncoder(analogID);
        turnEncoder.setPositionOffset(baseAngle);
        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setVelocityConversionFactor(Constants.encoderRotationToMeters);
        driveEncoder.setPositionConversionFactor(42 * Constants.encoderRotationToMeters);
        
        turnPID = new PIDController(P, 0, 0);
        // we don't use I or D since P works well enough
        turnPID.enableContinuousInput(0, 360);
        turnPID.setTolerance(turnSetpointTolerance, turnVelocityTolerance);
        // determined from a SYSID scan
        drivePID = new ProfiledPIDController(.11, 0, .015, constraints);
        drivePID.setTolerance(driveSetpointTolerance);

    }

    // runs while the bot is running
    @Override
    public void periodic() {
    }

    SlewRateLimiter accelerationLimiter = new SlewRateLimiter(30.0, -Constants.maxAcceleration, 0);

    public void setStates(SwerveModuleState state, boolean locked) {
        state = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(turnEncoder.getPos()));
        setAngle(state.angle.getDegrees());
        setDriveSpeed(accelerationLimiter.calculate(state.speedMetersPerSecond));
        NetworkTableInstance.getDefault().getTable("Speed").getEntry(moduleID).setDouble(state.speedMetersPerSecond);
    }

    public void setAngle(double angle) {
        turnPID.setSetpoint(angle);
        turnMotor.set(-turnPID.calculate(turnEncoder.getPos()));
    }

    public void setDriveSpeed(double velocity) {
        drivePID.setGoal(new State(velocity, 0));
        driveMotor.setVoltage(driveFeedforward.calculate(velocity) + drivePID.calculate(driveEncoder.getVelocity()));
        NetworkTableInstance.getDefault().getTable(moduleID).getEntry("Set Speed").setDouble(velocity);
        NetworkTableInstance.getDefault().getTable(moduleID).getEntry("Actual Speed")
                .setDouble(driveEncoder.getVelocity());
        
    }

    public void setTurnSpeed(double speed) {
        speed = Math.max(Math.min(speed, Constants.maxTurnSpeed), -Constants.maxTurnSpeed);
        turnMotor.set(speed);
    }

    public SwerveModulePosition getSwerveModulePosition() {
        double angle = turnEncoder.getPos();
        double distance = driveEncoder.getPosition();
        return new SwerveModulePosition(distance, new Rotation2d(3.14 * angle / 180));
    }

    public RelativeEncoder getDriveEncoder() {
        return this.driveEncoder;
    }

    public AbsoluteEncoder getTurnEncoder() {
        return this.turnEncoder;
    }

    public String getModuleID() {
        return this.moduleID;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(),
                Rotation2d.fromDegrees(turnEncoder.getPos()));
    }
}