package frc.robot.subsystems.Drive;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Swerve;



public class SwerveModule extends SubsystemBase {
    // TODO https://pro.docs.ctr-electronics.com/en/latest/docs/migration/migration-guide/closed-loop-guide.html
    // TODO https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/trapezoidal-profiles.html

    private final TalonFX mDriveMotor;
    private final TalonFX mSteerMotor;
    private final CANcoder mSteerEncoder;
    private final VelocityVoltage mVoltageVelocity;
    private final PositionVoltage mVoltagePosition;


    private SwerveModuleState mModuleState; // current state of the module without steer offset
    private double mModuleOffset;


    /**
     * @param driveMotorCANID
     * CANID of the Drive Motor (The Falcon motor that controls the wheel)
     * @param steerMotorCANID
     * CANID of the Steer Motor (The Falcon motor that controls turning)
     * @param steerEncoderCANID
     * CANID of the Steer Encoder (on-axis)
     * @param moduleOffsetDegrees
     * The Offset of the module (Relative to the robot)
     */
    public SwerveModule(int driveMotorCANID, int steerMotorCANID, int steerEncoderCANID, double moduleOffsetDegrees) {
        this.mDriveMotor = new TalonFX(driveMotorCANID);
        this.mSteerMotor = new TalonFX(steerMotorCANID);
        this.mSteerEncoder = new CANcoder(steerEncoderCANID);
        this.mVoltageVelocity = new VelocityVoltage(0);
        this.mVoltagePosition = new PositionVoltage(0);
        this.mModuleState = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
        this.mModuleOffset = moduleOffsetDegrees;
        configMotors(steerEncoderCANID);
        setNeutralMode(NeutralModeValue.Brake);
    }

    /**
     * Adds voltage limits (safety), and true ratios to the motors
     * @param steerEncoderCANID
     * The CANID of the encoder that will replace the included encoder inside the Falcon 500
     */
    private void configMotors(int steerEncoderCANID) {
        VoltageConfigs voltageConfigs = new VoltageConfigs()
            .withPeakForwardVoltage(Swerve.Stats.MAX_VOLTAGE)
            .withPeakReverseVoltage(-Swerve.Stats.MAX_VOLTAGE);
        CurrentLimitsConfigs statorConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(Swerve.Stats.STATOR_CURRENT_LIMIT)
            .withSupplyCurrentLimit(Swerve.Stats.SUPPLY_CURRENT_LIMIT);
        FeedbackConfigs feedbackConfigs = new FeedbackConfigs()
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
            .withFeedbackRemoteSensorID(steerEncoderCANID)
            .withRotorToSensorRatio(Swerve.Stats.ROTOR_TO_SENSOR_RATIO);
        Slot0Configs slot0DriveConfigs = new Slot0Configs()
            .withKA(Swerve.PID.Drive.A) // Acceleration 
            .withKS(Swerve.PID.Drive.S) // Static Friction Offset
            .withKV(Swerve.PID.Drive.V) // Velocity Feedforward
            // P I D
            .withKP(Swerve.PID.Drive.P)  // Proportional tuning - error
            .withKI(Swerve.PID.Drive.I) // Integral tuning - learning
            .withKD(Swerve.PID.Drive.D); // Derivative tuning - overshoot
        Slot0Configs slot0SteerConfigs = new Slot0Configs()
            .withKA(Swerve.PID.Steer.A) // Acceleration 
            .withKS(Swerve.PID.Steer.S) // Static Friction Offset
            .withKV(Swerve.PID.Steer.V) // Velocity Feedforward
            // P I D
            .withKP(Swerve.PID.Steer.P)  // Proportional tuning - error
            .withKI(Swerve.PID.Steer.I) // Integral tuning - learning
            .withKD(Swerve.PID.Steer.D); // Derivative tuning - overshoot
        
        MagnetSensorConfigs sensorConfigs = new MagnetSensorConfigs()
            .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1);
        
        ClosedLoopGeneralConfigs closedLoopConfigs = new ClosedLoopGeneralConfigs();
        closedLoopConfigs.ContinuousWrap = true;
            

        this.mDriveMotor.getConfigurator().apply(voltageConfigs);
        this.mDriveMotor.getConfigurator().apply(statorConfigs);
        this.mDriveMotor.getConfigurator().apply(slot0DriveConfigs);
        
        
        this.mSteerMotor.getConfigurator().apply(voltageConfigs);
        this.mSteerMotor.getConfigurator().apply(statorConfigs);
        this.mSteerMotor.getConfigurator().apply(feedbackConfigs);
        this.mSteerMotor.getConfigurator().apply(slot0SteerConfigs);
        this.mSteerMotor.getConfigurator().apply(closedLoopConfigs);

        this.mSteerEncoder.getConfigurator().apply(sensorConfigs);
        

        
    }

    /**
     * @param state 
     * The state of the Module (Affects both the Drive and Steer Motor)
     */
    public void setModuleState(SwerveModuleState state){
        //state = SwerveModuleState.optimize(state, this.mModuleState.angle); // CURRENTLY BREAKS THINGS
        setModuleVelocity(state.speedMetersPerSecond);
        setModuleAngle(state.angle.getDegrees());
        this.mModuleState = state;
    }

    /**
     * @param targetVelocity
     * The target velocity in meters per second
     */
    public void setModuleVelocity(double targetVelocity){
        this.mDriveMotor.setControl(mVoltageVelocity.withVelocity(mpsToRps(targetVelocity)));
    }

    /**
     * @param targetDegrees
     * The target angle in degrees of the module
     */
    public void setModuleAngle(double targetDegrees){
        this.mSteerMotor.setControl(this.mVoltagePosition.withPosition((this.mModuleOffset + targetDegrees)/360));
    }

    /**
     * Convertion from Rounds per minute to meters per second
     * @param value 
     * Rounds per minute
     */
    public double rpmToMps(double rpmValue) { 
        return (60/(2*Math.PI*Swerve.Stats.DRIVE_WHEEL_RADIUS_METERS))*rpmValue;
    }   

    /**
     * Convertion from Meters per second to rounds per minute
     * @param value 
     * Meters per second
     */
    public double mpsToRpm(double mpsValue) {
        return (mpsValue / (2 * Math.PI * Swerve.Stats.DRIVE_WHEEL_RADIUS_METERS)) * 60;
    }

    /**
     * Convertion from Meters per second to rounds per second
     * @param value 
     * Meters per second
     */
    public double mpsToRps(double mpsValue) {
        return (mpsValue / (2 * Math.PI * Swerve.Stats.DRIVE_WHEEL_RADIUS_METERS));
    }

    /**
     * @param neutralMode
     * Set's the NeutralMode of both motors of the module to CTRE'S NeutralMode
     * 
     * <i>(Check CTRE's NeutralMode documentation for more info)</i>
     *  
     */
    public void setNeutralMode(NeutralModeValue neutralMode) {
        this.mDriveMotor.setNeutralMode(neutralMode);
        this.mSteerMotor.setNeutralMode(neutralMode);
    }
    
    public Rotation2d getSteerAngle() {
        return new Rotation2d(this.mModuleOffset + this.mModuleState.angle.getDegrees());
    } 

    public void setCoast(){
        setNeutralMode(NeutralModeValue.Coast);
    }
    
    public void setBreak() {
        setNeutralMode(NeutralModeValue.Brake);
    }

    public TalonFX getDriveMotor() {
        return this.mDriveMotor;
    }

    public TalonFX getSteerMotor() {
        return this.mSteerMotor;
    }

    public CANcoder getSteerEncoder() {
        return this.mSteerEncoder;
    }

    public double getVelocityMetersPerSecond() {
        return this.mModuleState.speedMetersPerSecond;
    }

    public SwerveModuleState getModuleState() {
        return this.mModuleState;
    }


    /**
     *
     * @param speed The speed to set (Percentage). Value should be between -1.0 and 1.0.
     */
    public void setDriveMotor(double speed) {
        
        this.mDriveMotor.set(speed);
    }


    /**
     *
     * @param speed The speed to set (Percentage). Value should be between -1.0 and 1.0.
     */
    public void setSteerMotor(double speed) {
        this.mSteerMotor.set(speed);
    }

    @Override
    public void periodic() { // todo logs needed - ShuffleBoard
        this.mModuleState.angle = Rotation2d.fromDegrees(this.mSteerEncoder.getAbsolutePosition().getValueAsDouble() / 360);
        this.mModuleState.speedMetersPerSecond = rpmToMps(this.mDriveMotor.getRotorVelocity().getValueAsDouble() * 60);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
