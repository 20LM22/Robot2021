package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ExternalFollower;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.VecBuilder;
import frc.robot.Constants.DriveConstants;
import frc.robot.ShuffleboardLogging;

public class DriveSubsystem extends SubsystemBase implements ShuffleboardLogging {

        private Field2d m_field = new Field2d();

        // so this is the code where the talons are the masters
        private final TalonSRX m_masterLeft = new TalonSRX(DriveConstants.kMasterLeftPort);
        private final TalonSRX m_masterRight = new TalonSRX(DriveConstants.kMasterRightPort);
        private final CANSparkMax m_followerLeft = new CANSparkMax(DriveConstants.kFollowerLeftPort,
                        MotorType.kBrushless);
        private final CANSparkMax m_followerRight = new CANSparkMax(DriveConstants.kFollowerRightPort,
                        MotorType.kBrushless);

        // //so this is the code where the can spark maxes are the masters
        // private final CANSparkMax m_masterLeft = new
        // CANSparkMax(DriveConstants.kMasterLeftPort, MotorType.kBrushless);
        // private final CANSparkMax m_masterRight = new
        // CANSparkMax(DriveConstants.kMasterRightPort, MotorType.kBrushless);
        // private final TalonSRX m_followerLeft = new
        // TalonSRX(DriveConstants.kFollowerLeftPort);
        // private final TalonSRX m_followerRight = new
        // TalonSRX(DriveConstants.kFollowerRightPort);

        // private final CANEncoder m_leftEncoder = m_masterLeft.getEncoder();
        // private final CANEncoder m_rightEncoder = m_masterRight.getEncoder();
        // private final CANPIDController m_leftPIDController = m_masterLeft.getPIDController();
        // private final CANPIDController m_rightPIDController = m_masterRight.getPIDController();

        private final AHRS m_gyro = new AHRS(DriveConstants.kGyroPort);
        private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
                        Rotation2d.fromDegrees(getHeading()));

        // These represent our regular encoder objects, which we would
        // create to use on a real robot.
        // private Encoder m_lEncoder = new Encoder(2, 3);
        // private Encoder m_rEncoder = new Encoder(4, 5);

        // These are our EncoderSim objects, which we will only use in
        // simulation. However, you do not need to comment out these
        // declarations when you are deploying code to the roboRIO.
        // private EncoderSim m_leftEncoderSim = new EncoderSim(m_lEncoder);
        // private EncoderSim m_rightEncoderSim = new EncoderSim(m_rEncoder);

        // Create our gyro object like we would on a real robot.
        // private AnalogGyro m_gyro1 = new AnalogGyro(1);

        // Create the simulated gyro object, used for setting the gyro
        // angle. Like EncoderSim, this does not need to be commented out
        // when deploying code to the roboRIO.
        // private AnalogGyroSim m_gyroSim = new AnalogGyroSim(m_gyro1);

        // Create the simulation model of our drivetrain.
        private DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim(
                        // Create a linear system from our characterization gains.
                        LinearSystemId.identifyDrivetrainSystem(DriveConstants.kvVoltSecondsPerMeter,
                                        DriveConstants.kaVoltSecondsSquaredPerMeter, 2.3, .6),
                        DCMotor.getNEO(2), // 2 NEO motors on each side of the drivetrain.
                        DriveConstants.kGearRatio, // 7.29:1 gearing reduction.
                        DriveConstants.kTrackwidthMeters, // The track width is ... meters.
                        DriveConstants.kWheelDiameterMeters / 2,
                        // The standard deviations for measurement noise:
                        // x and y: 0.001 m
                        // heading: 0.001 rad
                        // l and r velocity: 0.1 m/s
                        // l and r position: 0.005 m
                        VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));

        /**
         * Initializes a new instance of the {@link DriveSubsystem} class.
         */
        public DriveSubsystem() {
                SmartDashboard.putData(m_field);

                // If the talons are the masters
                m_masterRight.configFactoryDefault();
                m_masterRight.setInverted(DriveConstants.kMasterRightInvert);
                m_masterRight.setNeutralMode(NeutralMode.Brake);
                m_masterRight.configVoltageCompSaturation(DriveConstants.kVoltageComp); 
                m_masterRight.enableVoltageCompensation(DriveConstants.kEnableVoltageComp);
                // // maybe a current limit?
                // m_masterRight.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,
                // DriveConstants.kSmartCurrentLimit, DriveConstants.kPeakCurrentLimit,
                // DriveConstants.kPeakCurrentDurationMillis));
                m_masterRight.configOpenloopRamp(DriveConstants.kRampRate);

                m_masterLeft.configFactoryDefault();
                m_masterLeft.setInverted(DriveConstants.kMasterLeftInvert);
                m_masterLeft.setNeutralMode(NeutralMode.Brake);
                m_masterLeft.configVoltageCompSaturation(DriveConstants.kVoltageComp); 
                m_masterLeft.enableVoltageCompensation(DriveConstants.kEnableVoltageComp);
                // // maybe a current limit?
                // m_masterLeft.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,
                // DriveConstants.kSmartCurrentLimit, DriveConstants.kPeakCurrentLimit,
                // DriveConstants.kPeakCurrentDurationMillis));
                m_masterLeft.configOpenloopRamp(DriveConstants.kRampRate); 
        
                m_followerLeft.restoreFactoryDefaults();
                m_followerLeft.setInverted(DriveConstants.kFollowerLeftOppose);
                m_followerLeft.setIdleMode(IdleMode.kCoast);
                m_followerLeft.enableVoltageCompensation(12);
                m_followerLeft.setSmartCurrentLimit(DriveConstants.kSmartCurrentLimit);
                m_followerLeft.setSecondaryCurrentLimit(DriveConstants.kPeakCurrentLimit,
                                DriveConstants.kPeakCurrentDurationMillis);
                m_followerLeft.setOpenLoopRampRate(DriveConstants.kRampRate);

                m_followerRight.restoreFactoryDefaults();
                m_followerRight.setInverted(DriveConstants.kFollowerRightOppose);
                m_followerRight.setIdleMode(IdleMode.kCoast);
                m_followerRight.enableVoltageCompensation(12);
                m_followerRight.setSmartCurrentLimit(DriveConstants.kSmartCurrentLimit);
                m_followerRight.setSecondaryCurrentLimit(DriveConstants.kPeakCurrentLimit,
                                DriveConstants.kPeakCurrentDurationMillis);
                m_followerRight.setOpenLoopRampRate(DriveConstants.kRampRate);

                m_followerLeft.follow(ExternalFollower.kFollowerPhoenix, m_masterLeft.getDeviceID());
                m_followerRight.follow(ExternalFollower.kFollowerPhoenix, m_masterRight.getDeviceID());

                // If the spark maxes are the masters
                // m_masterLeft.restoreFactoryDefaults();
                // m_masterLeft.setInverted(DriveConstants.kMasterLeftInvert);
                // m_masterLeft.setIdleMode(IdleMode.kBrake);
                // m_masterLeft.enableVoltageCompensation(12);
                // m_masterLeft.setSmartCurrentLimit(DriveConstants.kSmartCurrentLimit);
                // m_masterLeft.setSecondaryCurrentLimit(DriveConstants.kPeakCurrentLimit,
                //                 DriveConstants.kPeakCurrentDurationMillis);
                // m_masterLeft.setOpenLoopRampRate(DriveConstants.kRampRate);

                // m_masterRight.restoreFactoryDefaults();
                // m_masterRight.setInverted(DriveConstants.kMasterRightInvert);
                // m_masterRight.setIdleMode(IdleMode.kBrake);
                // m_masterRight.enableVoltageCompensation(12);
                // m_masterRight.setSmartCurrentLimit(DriveConstants.kSmartCurrentLimit);
                // m_masterRight.setSecondaryCurrentLimit(DriveConstants.kPeakCurrentLimit,
                //                 DriveConstants.kPeakCurrentDurationMillis);
                // m_masterRight.setOpenLoopRampRate(DriveConstants.kRampRate);

                // m_followerRight.configFactoryDefault();
                // m_followerRight.setInverted(DriveConstants.kFollowerRightOppose);
                // m_followerRight.setNeutralMode(NeutralMode.Coast);
                // m_followerRight.configVoltageCompSaturation(DriveConstants.kVoltageComp); 
                // m_followerRight.enableVoltageCompensation(DriveConstants.kEnableVoltageComp);
                // // // maybe a current limit?
                // // m_followerRight.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,
                // // DriveConstants.kSmartCurrentLimit, DriveConstants.kPeakCurrentLimit,
                // // DriveConstants.kPeakCurrentDurationMillis));
                // m_followerRight.configOpenloopRamp(DriveConstants.kRampRate);

                // m_followerLeft.configFactoryDefault();
                // m_followerLeft.setInverted(DriveConstants.kFollowerRightOppose);
                // m_followerLeft.setNeutralMode(NeutralMode.Coast);
                // m_followerLeft.configVoltageCompSaturation(DriveConstants.kVoltageComp); 
                // m_followerLeft.enableVoltageCompensation(DriveConstants.kEnableVoltageComp);
                // // // maybe a current limit?
                // // m_followerLeft.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,
                // // DriveConstants.kSmartCurrentLimit, DriveConstants.kPeakCurrentLimit,
                // // DriveConstants.kPeakCurrentDurationMillis));
                // m_followerLeft.configOpenloopRamp(DriveConstants.kRampRate);

                // //m_followerRight.follow(m_masterRight); //This doesn't work...
                // //m_followerLeft.follow(m_masterLeft); //This doesn't work...

                //so this stuff should be handled directly by the talon motor controllers
                // m_leftEncoder.setPositionConversionFactor(
                //                 (1 / DriveConstants.kGearRatio) * Math.PI * DriveConstants.kWheelDiameterMeters);
                // m_leftEncoder.setVelocityConversionFactor(
                //                 (1 / DriveConstants.kGearRatio) * Math.PI * DriveConstants.kWheelDiameterMeters / 60.0);

                // m_rightEncoder.setPositionConversionFactor(
                //                 (1 / DriveConstants.kGearRatio) * Math.PI * DriveConstants.kWheelDiameterMeters);
                // m_rightEncoder.setVelocityConversionFactor(
                //                 (1 / DriveConstants.kGearRatio) * Math.PI * DriveConstants.kWheelDiameterMeters / 60.0);

                // simulated encoders!
                // m_lEncoder.setDistancePerPulse(2 * Math.PI * DriveConstants.kWheelDiameterMeters / 2 / 1000); // TODO
                // m_rEncoder.setDistancePerPulse(2 * Math.PI * DriveConstants.kWheelDiameterMeters / 2 / 1000);

                // m_leftPIDController.setP(DriveConstants.kP);
                // m_leftPIDController.setI(DriveConstants.kI);
                // m_leftPIDController.setIZone(DriveConstants.kIz);
                // m_leftPIDController.setD(DriveConstants.kD);
                // m_leftPIDController.setFF(DriveConstants.kFF);
                // m_leftPIDController.setOutputRange(DriveConstants.kMinOutput, DriveConstants.kMaxOutput);
                // m_leftPIDController.setFeedbackDevice(m_leftEncoder);

                // m_rightPIDController.setP(DriveConstants.kP);
                // m_rightPIDController.setI(DriveConstants.kI);
                // m_rightPIDController.setIZone(DriveConstants.kIz);
                // m_rightPIDController.setD(DriveConstants.kD);
                // m_rightPIDController.setFF(DriveConstants.kFF);
                // m_rightPIDController.setOutputRange(DriveConstants.kMinOutput, DriveConstants.kMaxOutput);
                // m_rightPIDController.setFeedbackDevice(m_rightEncoder);


                //setting up the encoder stuff for this saturday
                m_masterLeft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
                m_masterRight.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

                m_masterLeft.setSelectedSensorPosition(0);
                m_masterRight.setSelectedSensorPosition(0);
        
                m_masterLeft.setSensorPhase(DriveConstants.kLeftSensorPhase);
                m_masterRight.setSensorPhase(DriveConstants.kRightSensorPhase);

                resetOdometry(new Pose2d(0, 0, new Rotation2d()));
        }

        /**
         * Update odometry
         */
        public void periodic() {
                SmartDashboard.putNumber("Left wheel", getLeftEncoderPosition());
                SmartDashboard.putNumber("Right wheel", getRightEncoderPosition());
                SmartDashboard.putNumber("Heading",
                m_odometry.getPoseMeters().getRotation().getDegrees());
                m_odometry.update(Rotation2d.fromDegrees(getHeading()),
                getLeftEncoderPosition(), getRightEncoderPosition());

                // m_odometry.update(m_gyro1.getRotation2d(), m_lEncoder.getDistance(), m_rEncoder.getDistance());
                // m_field.setRobotPose(m_odometry.getPoseMeters());
        }

        // public void simulationPeriodic() {
        //         // Set the inputs to the system. Note that we need to convert
        //         // the [-1, 1] PWM signal to voltage by multiplying it by the
        //         // robot controller voltage.
        //         m_driveSim.setInputs(m_masterLeft.get() * RobotController.getInputVoltage(),
        //                         m_masterRight.get() * RobotController.getInputVoltage());

        //         // Advance the model by 20 ms. Note that if you are running this
        //         // subsystem in a separate thread or have changed the nominal timestep
        //         // of TimedRobot, this value needs to match it.
        //         m_driveSim.update(0.02);

        //         // Update all of our sensors.
        //         m_leftEncoderSim.setDistance(m_driveSim.getLeftPositionMeters());
        //         m_leftEncoderSim.setRate(m_driveSim.getLeftVelocityMetersPerSecond());
        //         m_rightEncoderSim.setDistance(m_driveSim.getRightPositionMeters());
        //         m_rightEncoderSim.setRate(m_driveSim.getRightVelocityMetersPerSecond());
        //         m_gyroSim.setAngle(-m_driveSim.getHeading().getDegrees());
        // }

        /**
         * @return The left encoder position (meters)
         */
        public double getLeftEncoderPosition() {
                //return m_leftEncoder.getPosition();
                return m_masterLeft.getSelectedSensorPosition();
        }

        /**
         * @return The right encoder position (meters)
         */
        public double getRightEncoderPosition() {
                //return -m_rightEncoder.getPosition();
                return m_masterRight.getSelectedSensorPosition(); //TODO might need to negate?
        }

        /**
         * @return The average encoder distance of both encoders (meters)
         */
        public double getAverageEncoderDistance() {
                //return (getLeftEncoderPosition() + getRightEncoderPosition()) / 2.0;
                return (m_masterRight.getSelectedSensorPosition() + m_masterLeft.getSelectedSensorPosition()) / 2.0;
        }

        /**
         * @return The velocity of the left encoder (meters/s)
         */
        public double getLeftEncoderVelocity() {
                // return m_leftEncoder.getVelocity();
                return m_masterLeft.getSelectedSensorVelocity();
        }

        /**
         * @return The velocity of the right encoder (meters/s)
         */
        public double getRightEncoderVelocity() {
                // return -m_rightEncoder.getVelocity();
                return m_masterRight.getSelectedSensorVelocity(); //TODO might need to negate
        }

        /**
         * @return Pose of the robot
         */
        public Pose2d getPose() {
                return m_odometry.getPoseMeters();
        }

        /**
         * @return Wheel speeds of the robot
         */
        public DifferentialDriveWheelSpeeds getWheelSpeeds() {
                return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity(), getRightEncoderVelocity());
        }

        /**
         * @return The heading of the gyro (degrees)
         */
        public double getHeading() {
                return m_gyro.getYaw() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
        }

        /**
         * @return The rate of the gyro turn (deg/s)
         */
        public double getTurnRate() {
                return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
        }

        /**
         * Sets both encoders to 0
         */
        public void resetEncoders() {
                // m_leftEncoder.setPosition(0);
                // m_rightEncoder.setPosition(0);
                // m_lEncoder.reset();
                // m_rEncoder.reset();

                // this is if the talons are the masters
                m_masterLeft.setSelectedSensorPosition(0);
                m_masterRight.setSelectedSensorPosition(0);
        }

        /**
         * @param pose Pose to set the robot to
         */
        public void resetOdometry(Pose2d pose) {
                resetEncoders();
                m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
        }

        /**
         * @param straight Straight percent output
         * @param left     Left percent output
         * @param right    Right percent output
         */
        public void arcadeDrive(double straight, double left, double right) {
                tankDrive(DriveConstants.kSpeedLimitFactor * (straight - left + right),
                                DriveConstants.kSpeedLimitFactor * (straight + left - right));
        }

        /**
         * @param leftSpeed  Left motors percent output
         * @param rightSpeed Right motors percent output
         */
        public void tankDrive(double leftSpeed, double rightSpeed) {
                
                //this is if the talons are the masters
                m_masterLeft.set(ControlMode.PercentOutput, leftSpeed);
                m_masterRight.set(ControlMode.PercentOutput, rightSpeed);

                // //this is if the spark maxes are the masters
                // m_masterLeft.set(leftSpeed);
                // m_masterRight.set(rightSpeed);
        }

        public void setWheelSpeeds(DifferentialDriveWheelSpeeds wheelSpeeds) {
              
                // this is if the talons are the masters TODO should be updated to work with PIDs
                //ok, so this one might work
                m_masterRight.set(ControlMode.Velocity, DriveConstants.kFeedForward.calculate(wheelSpeeds.leftMetersPerSecond));
                m_masterLeft.set(ControlMode.Velocity, DriveConstants.kFeedForward.calculate(wheelSpeeds.rightMetersPerSecond));             

                // // this is what last year's code was doing to set the wheel speeds with SPARK MAXES irl
                // m_leftPIDController.setReference(wheelSpeeds.leftMetersPerSecond,
                // ControlType.kVelocity, DriveConstants.kSlotID,
                // DriveConstants.kFeedForward.calculate(wheelSpeeds.leftMetersPerSecond));
                // m_rightPIDController.setReference(wheelSpeeds.rightMetersPerSecond,
                // ControlType.kVelocity,
                // DriveConstants.kSlotID,
                // DriveConstants.kFeedForward.calculate(wheelSpeeds.rightMetersPerSecond));

                // //this is what the simulation is using WITH SPARK MAXES to set the speeds of the wheels
                // m_masterRight.set(DriveConstants.kFeedForward.calculate(wheelSpeeds.leftMetersPerSecond));
                // m_masterLeft.set(DriveConstants.kFeedForward.calculate(wheelSpeeds.rightMetersPerSecond));
        }

        public void configureShuffleboard() {
                ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Drive");
                shuffleboardTab.addNumber("Left speed", () -> getWheelSpeeds().leftMetersPerSecond).withSize(4, 2)
                                .withPosition(0, 0).withWidget(BuiltInWidgets.kGraph);
                shuffleboardTab.addNumber("Right speed", () -> getWheelSpeeds().rightMetersPerSecond).withSize(4, 2)
                                .withPosition(4, 0).withWidget(BuiltInWidgets.kGraph);
                shuffleboardTab.addNumber("Left motor speed", () -> getLeftEncoderPosition()).withSize(1, 1)
                                .withPosition(0, 2).withWidget(BuiltInWidgets.kTextView);
                shuffleboardTab.addNumber("Right motor speed", () -> getRightEncoderPosition()).withSize(1, 1)
                                .withPosition(1, 2).withWidget(BuiltInWidgets.kTextView);
                shuffleboardTab.addNumber("Heading", () -> getHeading()).withSize(1, 1).withPosition(2, 2)
                                .withWidget(BuiltInWidgets.kTextView);
        }
}