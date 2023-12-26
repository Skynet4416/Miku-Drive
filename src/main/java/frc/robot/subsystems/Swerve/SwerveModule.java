package frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Swerve;
import edu.wpi.first.wpilibj.shuffleboard.*;



public class SwerveModule extends SubsystemBase {
    // TODO https://pro.docs.ctr-electronics.com/en/latest/docs/migration/migration-guide/closed-loop-guide.html
    // TODO https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/trapezoidal-profiles.html

    private final TalonFX m_driveMotor;
    private final TalonFX m_steerMotor;
    private final CANcoder m_steerEncoder;
    private final VelocityVoltage m_voltageVelocity;
    private final PositionVoltage m_voltagePosition;

    private final VelocityDutyCycle m_velocityDutyCycle;
    private final MotionMagicDutyCycle m_motionMagicDutyCycle;

    private SwerveModuleState m_moduleState; // current state of the module without steer offset
    private double m_moduleOffset;


    /**
     * @param driveMotorCANID
     * CANID of the Drive Motor (The Falcon motor that controls the wheel)
     * @param steerMotorCANID
     * CANID of the Steer Motor (The Falcon motor that controls turning)
     * @param steerEncoderCANID
     * CANID of the Steer Encoder (on-axis)
     * @param moduleOffsetInDegrees
     * The Offset of the module (Relative to the robot)
     */
    public SwerveModule(int driveMotorCANID, int steerMotorCANID, int steerEncoderCANID, double moduleOffsetInDegrees) {
        this.m_driveMotor = new TalonFX(driveMotorCANID);
        this.m_steerMotor = new TalonFX(steerMotorCANID);
        this.m_steerEncoder = new CANcoder(steerEncoderCANID);
        
        this.m_velocityDutyCycle = new VelocityDutyCycle(0);
        this.m_motionMagicDutyCycle = new MotionMagicDutyCycle(0);
        
        this.m_voltageVelocity = new VelocityVoltage(0,0,false,0,0,false);
        this.m_voltagePosition = new PositionVoltage(0,0,false,0,0,false);
        this.m_moduleState = new SwerveModuleState(0, Rotation2d.fromDegrees(0));

        this.m_moduleOffset = moduleOffsetInDegrees;
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
            .withPeakForwardVoltage(Swerve.Stats.kMaxVoltage)
            .withPeakReverseVoltage(-Swerve.Stats.kMaxVoltage);

        CurrentLimitsConfigs statorConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(35)
            .withSupplyCurrentLimit(35);

        FeedbackConfigs feedbackConfigs = new FeedbackConfigs()
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
            .withFeedbackRemoteSensorID(steerEncoderCANID)
            .withRotorToSensorRatio(Swerve.Stats.kRotorToSensorRatio);

        Slot0Configs slot0DriveConfigs = new Slot0Configs()
            .withKA(Swerve.PID.Drive.kA) // Acceleration 
            .withKS(Swerve.PID.Drive.kS) // Static Friction Offset
            .withKV(Swerve.PID.Drive.kV) // Velocity Feedforward
            // P I D
            .withKP(Swerve.PID.Drive.kP)  // Proportional tuning - error
            .withKI(Swerve.PID.Drive.kI) // Integral tuning - learning
            .withKD(Swerve.PID.Drive.kD); // Derivative tuning - overshoot



        Slot0Configs slot0SteerConfigs = new Slot0Configs()
            .withKA(Swerve.PID.Steer.kA) // Acceleration 
            .withKS(Swerve.PID.Steer.kS) // Static Friction Offset
            .withKV(Swerve.PID.Steer.kV) // Velocity Feedforward
            // P I D
            .withKP(Swerve.PID.Steer.kP)  // Proportional tuning - error
            .withKI(Swerve.PID.Steer.kI) // Integral tuning - learning
            .withKD(Swerve.PID.Steer.kD); // Derivative tuning - overshoot
        


        this.m_driveMotor.getConfigurator().apply(voltageConfigs);
        this.m_driveMotor.getConfigurator().apply(statorConfigs);
        this.m_driveMotor.getConfigurator().apply(slot0DriveConfigs);
        
        
        this.m_steerMotor.getConfigurator().apply(voltageConfigs);
        this.m_steerMotor.getConfigurator().apply(statorConfigs);
        this.m_steerMotor.getConfigurator().apply(feedbackConfigs);
        this.m_steerMotor.getConfigurator().apply(slot0SteerConfigs);
        

        
    }


    /**
     * @param degrees 
     * The target angle in degrees of the module
     */
    public void setModuleAngle(double degrees){
        this.m_steerMotor.setControl(this.m_voltagePosition.withPosition((this.m_moduleOffset - Math.min(180,Math.max(degrees,-180)))/360));
    }

    /**
     * @param state 
     * The state of the Module (Affects both the Drive and Steer Motor)
     */
    public void setModuleState(SwerveModuleState state){
        //state = SwerveModuleState.optimize(state, this.m_moduleState.angle); // CURRENTLY BREAKS THINGS
        this.m_driveMotor.setControl(m_voltageVelocity.withVelocity(mpsToRps(state.speedMetersPerSecond)));
        this.m_steerMotor.setControl(this.m_voltagePosition.withPosition((this.m_moduleOffset + state.angle.getDegrees())/360));
    }

    /**
     * Convertion from Rounds per minute to meters per second
     * @param value 
     * Rounds per minute
     */
    public double rpmToMps(double rpmValue) { 
        return (60/(2*Math.PI*Swerve.Stats.wheelRadiusMeters))*rpmValue;
    }   

    /**
     * Convertion from Meters per second to rounds per minute
     * @param value 
     * Meters per second
     */
    public double mpsToRpm(double mpsValue) {
        return (mpsValue / (2 * Math.PI * Swerve.Stats.wheelRadiusMeters)) * 60;
    }

    /**
     * Convertion from Meters per second to rounds per second
     * @param value 
     * Meters per second
     */
    public double mpsToRps(double mpsValue) {
        return (mpsValue / (2 * Math.PI * Swerve.Stats.wheelRadiusMeters));
    }

    /**
     * @param neutralMode
     * Set's the NeutralMode of both motors of the module to CTRE'S NeutralMode
     * 
     * <i>(Check CTRE's NeutralMode documentation for more info)</i>
     *  
     */
    public void setNeutralMode(NeutralModeValue neutralMode) {
        this.m_driveMotor.setNeutralMode(neutralMode);
        this.m_steerMotor.setNeutralMode(neutralMode);
    }
    
    public Rotation2d getSteerAngle() {
        return this.m_moduleState.angle;
    } 

    public void setCoast(){
        setNeutralMode(NeutralModeValue.Coast);
    }
    
    public void setBreak() {
        setNeutralMode(NeutralModeValue.Brake);
    }

    public TalonFX getDriveMotor() {
        return this.m_driveMotor;
    }

    public TalonFX getSteerMotor() {
        return this.m_steerMotor;
    }

    public CANcoder getSteerEncoder() {
        return this.m_steerEncoder;
    }

    public double getVelocityMetersPerSecond() {
        return this.m_moduleState.speedMetersPerSecond;
    }

    public SwerveModuleState getModuleState() {
        return this.m_moduleState;
    }


    /**
     *
     * @param speed The speed to set (Percentage). Value should be between -1.0 and 1.0.
     */
    public void setDriveMotor(double speed) {
        
        this.m_driveMotor.set(speed);
    }


    /**
     *
     * @param speed The speed to set (Percentage). Value should be between -1.0 and 1.0.
     */
    public void setSteerMotor(double speed) {
        this.m_steerMotor.set(speed);
    }

    @Override
    public void periodic() { // todo logs needed - ShuffleBoard
        this.m_moduleState.angle = Rotation2d.fromDegrees(this.m_steerEncoder.getAbsolutePosition().getValueAsDouble() / 360);
        this.m_moduleState.speedMetersPerSecond = rpmToMps(this.m_driveMotor.getRotorVelocity().getValueAsDouble() * 60);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
