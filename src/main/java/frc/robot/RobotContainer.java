package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import com.ctre.phoenix.CANifier.LEDChannel;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArduinoConstants.LEDColorValues;
import frc.robot.Constants.ArduinoConstants.MainLEDModes;
import frc.robot.Constants.ArduinoConstants.ShooterLEDModes;
import frc.robot.Constants.CarouselConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ControllerConstants.Axis;
import frc.robot.Constants.ControllerConstants.Button;
import frc.robot.Constants.ControllerConstants.DPad;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldLocation;
import frc.robot.Constants.LoggingConstants;
import frc.robot.commands.arduinocommands.UpdateLEDsCommand;
import frc.robot.commands.armcommands.BounceArmCommand;
import frc.robot.commands.armcommands.DriveArmCommand;
import frc.robot.commands.armcommands.ExtendArmCommand;
import frc.robot.commands.armcommands.RetractArmCommand;
import frc.robot.commands.autocommands.ShootForwardCG;
import frc.robot.commands.carouselcommands.AutoSpeedCarouselCommand;
import frc.robot.commands.carouselcommands.RunCarouselCommand;
import frc.robot.commands.carouselcommands.ToOpenSpaceCommand;
import frc.robot.commands.climbercommands.DriveScissorsCommand;
import frc.robot.commands.drivecommands.ArcadeDriveCommand;
import frc.robot.commands.drivecommands.BackupCommand;
import frc.robot.commands.drivecommands.LimelightTurnCommand;
import frc.robot.commands.drivecommands.PixyGalacticCommand;
import frc.robot.commands.drivecommands.PixyTargetCommand;
import frc.robot.commands.drivecommands.TrajectoryFollowCommand;
import frc.robot.commands.feedercommands.AutoFeederCommand;
import frc.robot.commands.feedercommands.FeederCommand;
import frc.robot.commands.feedercommands.ReverseFeederCommand;
import frc.robot.commands.intakecommands.IntakeCommand;
import frc.robot.commands.intakecommands.OuttakeCommand;
import frc.robot.commands.shootcommands.DriveHoodCommand;
import frc.robot.commands.shootcommands.HoodPositionCommand;
import frc.robot.commands.shootcommands.ShootSetupCommand;
import frc.robot.subsystems.ArduinoSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CarouselSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class RobotContainer {
	// subsystems
	private final ArduinoSubsystem m_arduinoSubsystem = new ArduinoSubsystem();
	private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
	private final CarouselSubsystem m_carouselSubsystem = new CarouselSubsystem();
	private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
	private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
	private final FeederSubsystem m_feederSubsystem = new FeederSubsystem();
	private final FlywheelSubsystem m_flywheelSubsystem = new FlywheelSubsystem();
	private final HoodSubsystem m_hoodSubsystem = new HoodSubsystem();
	private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
	private final LimelightSubsystem m_limelightSubsystem = new LimelightSubsystem();
	// controllers
	private final Joystick m_driverController = new Joystick(ControllerConstants.kDriverControllerPort);
	private final Joystick m_operatorController = new Joystick(ControllerConstants.kOperatorControllerPort);
	// auto selector
	private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();
	private final ShuffleboardLogging[] m_subsystems = { m_arduinoSubsystem, m_armSubsystem, m_carouselSubsystem,
			m_climberSubsystem, m_driveSubsystem, m_feederSubsystem, m_flywheelSubsystem, m_hoodSubsystem,
			m_intakeSubsystem, m_limelightSubsystem };

	public RobotContainer() {

		m_limelightSubsystem.turnOffLight();
		m_autoChooser.addOption("Auto", new ShootForwardCG(m_driveSubsystem, m_flywheelSubsystem, m_hoodSubsystem,
				m_feederSubsystem, m_carouselSubsystem));
		SmartDashboard.putData(m_autoChooser);
		configureButtonBindings();
		// configureTestingBindings();
		configureShuffleboard();

		// Generate all trajectories at startup to prevent loop overrun
		generateAutonomousCommands();
		// LEDs

		m_arduinoSubsystem.setDefaultCommand(new UpdateLEDsCommand(m_arduinoSubsystem, () -> {
			return MainLEDModes.kTwinkling;
		}, () -> {
			return LEDColorValues.kGreen;
		}, () -> {
			return ShooterLEDModes.kTwinkling;
		}, () -> {
			return LEDColorValues.kGreen;
		}));
	}

	private void configureButtonBindings() {
		// Driver
		// Drive arcade
		m_driveSubsystem.setDefaultCommand(
				new ArcadeDriveCommand(m_driveSubsystem, () -> -m_driverController.getRawAxis(Axis.kLeftY),
						() -> (m_driverController.getRawAxis(Axis.kLeftTrigger) + 1) / 2,
						() -> (m_driverController.getRawAxis(Axis.kRightTrigger) + 1) / 2));
		// Climber
		m_climberSubsystem.setDefaultCommand(
				new DriveScissorsCommand(m_climberSubsystem, () -> -m_driverController.getRawAxis(Axis.kRightY)));
		// Carousel default
		m_carouselSubsystem.setDefaultCommand(new ToOpenSpaceCommand(m_carouselSubsystem));

		// Pixy ball follow
		new JoystickButton(m_driverController, Button.kX).whenHeld(new ParallelCommandGroup(
				new PixyTargetCommand(m_driveSubsystem, m_arduinoSubsystem,
						() -> -m_driverController.getRawAxis(Axis.kLeftY)),
				new RunCarouselCommand(m_carouselSubsystem, CarouselConstants.kIntakeVelocity),
				new IntakeCommand(m_intakeSubsystem), new BounceArmCommand(m_armSubsystem)));
		// Turn to target
		new JoystickButton(m_driverController, Button.kLeftBumper)
				.whenHeld(new LimelightTurnCommand(m_limelightSubsystem, m_driveSubsystem, .2))
				.whenHeld(new UpdateLEDsCommand(m_arduinoSubsystem, () -> {
					return MainLEDModes.kSolid;
				}, () -> {
					return LEDColorValues.kGreen;
				}, () -> {
					return ShooterLEDModes.kSolid;
				}, () -> {
					return LEDColorValues.kGreen;
				}));
		// Run carousel fast
		new JoystickButton(m_driverController, Button.kRightBumper)
				.whenHeld(new AutoSpeedCarouselCommand(m_carouselSubsystem, m_flywheelSubsystem::getSetpoint));
		// Feeder
		new POVButton(m_driverController, DPad.kUp).whenHeld(new FeederCommand(m_feederSubsystem));
		// Run carousel default speed
		new POVButton(m_driverController, DPad.kDown)
				.toggleWhenPressed(new RunCarouselCommand(m_carouselSubsystem, CarouselConstants.kVelocity));
		new POVButton(m_driverController, DPad.kRight).whenHeld(new ReverseFeederCommand(m_feederSubsystem));
		new POVButton(m_driverController, DPad.kLeft)
				.whenHeld(new BackupCommand(m_driveSubsystem, DriveConstants.kBackupDistance));

		// Operator
		// Intake
		new JoystickButton(m_operatorController, Button.kX)
				.whenHeld(new ParallelCommandGroup(new IntakeCommand(m_intakeSubsystem),
						new RunCarouselCommand(m_carouselSubsystem, CarouselConstants.kIntakeVelocity)))
				.whenHeld(new BounceArmCommand(m_armSubsystem)); // bounces the arm, spins the carousel, and turns on
																	// the intake wheels
		new JoystickButton(m_operatorController, Button.kCircle).whenHeld(new OuttakeCommand(m_intakeSubsystem)); // spins
																													// intake
																													// wheels
																													// backwards
		// Arm
		new JoystickButton(m_operatorController, Button.kLeftBumper).whenPressed(new RetractArmCommand(m_armSubsystem));
		new JoystickButton(m_operatorController, Button.kRightBumper).whenPressed(new ExtendArmCommand(m_armSubsystem));
		new Trigger(() -> Math.abs(
				(m_operatorController.getRawAxis(Axis.kLeftTrigger) + 1) / 2) > ControllerConstants.kTriggerDeadzone
				|| Math.abs(
						(m_operatorController.getRawAxis(Axis.kRightTrigger) + 1) / 2) > ControllerConstants.kDeadzone)
								.whenActive(new DriveArmCommand(m_armSubsystem,
										() -> (m_operatorController.getRawAxis(Axis.kLeftTrigger) + 1) / 2,
										() -> (m_operatorController.getRawAxis(Axis.kRightTrigger) + 1) / 2));
		m_armSubsystem.setDefaultCommand(
				new DriveArmCommand(m_armSubsystem, () -> (m_operatorController.getRawAxis(Axis.kLeftTrigger) + 1) / 2,
						() -> (m_operatorController.getRawAxis(Axis.kRightTrigger) + 1) / 2));
		// Carousel jostle
		new JoystickButton(m_operatorController, Button.kTriangle)
				.whenHeld(new RunCarouselCommand(m_carouselSubsystem, CarouselConstants.kJostleVelocity));
		// Hood and flywheel override
		new POVButton(m_operatorController, DPad.kDown).whenHeld(new ParallelCommandGroup(
				new ShootSetupCommand(m_flywheelSubsystem, m_hoodSubsystem, () -> FieldLocation.YELLOW), // WALL
				new AutoFeederCommand(m_feederSubsystem, m_carouselSubsystem::atOpenSpace,
						m_flywheelSubsystem::atSetpoint)));
		new POVButton(m_operatorController, DPad.kLeft).whenHeld(new ParallelCommandGroup(
				new ShootSetupCommand(m_flywheelSubsystem, m_hoodSubsystem, () -> FieldLocation.GREEN), // INITLINE
				new AutoFeederCommand(m_feederSubsystem, m_carouselSubsystem::atOpenSpace,
						m_flywheelSubsystem::atSetpoint)));
		new POVButton(m_operatorController, DPad.kUp).whenHeld(new ParallelCommandGroup(
				new ShootSetupCommand(m_flywheelSubsystem, m_hoodSubsystem, () -> FieldLocation.RED), // CLOSETRENCH
				new AutoFeederCommand(m_feederSubsystem, m_carouselSubsystem::atOpenSpace,
						m_flywheelSubsystem::atSetpoint)));
		new POVButton(m_operatorController, DPad.kRight).whenHeld(new ParallelCommandGroup(
				new ShootSetupCommand(m_flywheelSubsystem, m_hoodSubsystem, () -> FieldLocation.BLUE), // FARTWRENCH
				new AutoFeederCommand(m_feederSubsystem, m_carouselSubsystem::atOpenSpace,
						m_flywheelSubsystem::atSetpoint)));
		// Zero hood encoder
		new JoystickButton(m_operatorController, Button.kTrackpad)
				.and(new JoystickButton(m_operatorController, Button.kTriangle))
				.whenActive(() -> m_hoodSubsystem.resetEncoder(), m_hoodSubsystem);
		// Zero carousel encoder
		new JoystickButton(m_operatorController, Button.kTrackpad)
				.and(new JoystickButton(m_operatorController, Button.kCircle))
				.whenActive(() -> m_carouselSubsystem.resetEncoder(), m_carouselSubsystem);
		// Zero arm encoder
		new JoystickButton(m_operatorController, Button.kTrackpad)
				.and(new JoystickButton(m_operatorController, Button.kX))
				.whenActive(() -> m_armSubsystem.resetEncoder(), m_armSubsystem);
		// Manually drive carousel
		// new JoystickButton(m_operatorController, Button.kTrackpad).whileActiveOnce(
		// new DriveCarouselCommand(m_carouselSubsystem, () ->
		// -m_driverController.getRawAxis(Axis.kRightY)));

		// TODO change the zeroing of the arm encoder to a different button somewhere so
		// that we can be sure it's actually being zeroed
		new JoystickButton(m_operatorController, Button.kSquare).whenActive(() -> m_hoodSubsystem.resetEncoder(),
				m_hoodSubsystem); // when active = when pressed
	}

	private void configureTestingBindings() {
		// // driving
		// m_driveSubsystem.setDefaultCommand(
		// new ArcadeDriveCommand(m_driveSubsystem, () ->
		// -m_driverController.getRawAxis(Axis.kLeftY),
		// () -> (m_driverController.getRawAxis(Axis.kLeftTrigger) + 1) / 2,
		// () -> (m_driverController.getRawAxis(Axis.kRightTrigger) + 1) / 2));
		// // flywheel
		new POVButton(m_operatorController, DPad.kUp).whenPressed(() -> m_flywheelSubsystem.incrementSpeed(),
				m_flywheelSubsystem);
		new POVButton(m_operatorController, DPad.kDown).whenPressed(() -> m_flywheelSubsystem.decrementSpeed(),
				m_flywheelSubsystem);
		new POVButton(m_operatorController, DPad.kLeft).whenPressed(() -> m_flywheelSubsystem.setVelocity(5000));
		new POVButton(m_operatorController, DPad.kRight).whenPressed(() -> m_flywheelSubsystem.setVelocity(6000));
		new JoystickButton(m_operatorController, Button.kLeftBumper)
				.whenPressed(() -> m_flywheelSubsystem.setVelocity(0));
		// Hood and flywheel override
		// new POVButton(m_operatorController, DPad.kDown)
		// .whenHeld(new ShootSetupCommand(m_flywheelSubsystem, m_hoodSubsystem, () ->
		// FieldLocation.WALL));
		// new POVButton(m_operatorController, DPad.kLeft)
		// .whenHeld(new ShootSetupCommand(m_flywheelSubsystem, m_hoodSubsystem, () ->
		// FieldLocation.TWOFEET));
		// new POVButton(m_operatorController, DPad.kUp)
		// .whenHeld(new ShootSetupCommand(m_flywheelSubsystem, m_hoodSubsystem, () ->
		// FieldLocation.INITLINE));
		// new POVButton(m_operatorController, DPad.kRight)
		// .whenHeld(new ShootSetupCommand(m_flywheelSubsystem, m_hoodSubsystem, () ->
		// FieldLocation.CLOSETRENCH));
		// hood
		m_hoodSubsystem.setDefaultCommand(

				new DriveHoodCommand(m_hoodSubsystem, () -> m_operatorController.getRawAxis(Axis.kRightY) * 0.1));

		new JoystickButton(m_operatorController, Button.kOptions).whenPressed(() -> m_hoodSubsystem.resetEncoder());

		new JoystickButton(m_operatorController, Button.kRightBumper)
				.whenPressed(new HoodPositionCommand(m_hoodSubsystem, 0));
		// arm
		m_armSubsystem.setDefaultCommand(
				new DriveArmCommand(m_armSubsystem, () -> (m_operatorController.getRawAxis(Axis.kLeftTrigger) + 1) / 2,
						() -> (m_operatorController.getRawAxis(Axis.kRightTrigger) + 1) / 2));
		// intake
		new JoystickButton(m_operatorController, Button.kX).whenHeld(new IntakeCommand(m_intakeSubsystem));
		// carousel
		new JoystickButton(m_operatorController, Button.kCircle)
				.toggleWhenPressed(new RunCarouselCommand(m_carouselSubsystem, CarouselConstants.kVelocity));
		// feeder
		new JoystickButton(m_operatorController, Button.kTriangle).whenHeld(new FeederCommand(m_feederSubsystem));
	}

	public void configureShuffleboard() {
		for (int i = 0; i < m_subsystems.length; i++) {
			if (LoggingConstants.kSubsystems[i]) {
				m_subsystems[i].configureShuffleboard();
			}
		}
	}

	public Command getAutonomousCommand() {
		// return new TrajectoryFollowCommand(m_driveSubsystem,
		// TrajectoryGenerator.generateTrajectory(new Pose2d(0,0,new Rotation2d()),
		// List.of(
		// new Translation2d(2.5, 0), new Translation2d(3.5, -1.2), new
		// Translation2d(2.6, -1.6), new Translation2d(1.8, -1.2),
		// new Translation2d(2.5, 0), new Translation2d(5.5, -.3), new Translation2d(6,
		// 1.2), new Translation2d(5.5, 1.3),
		// new Translation2d(4, .9), new Translation2d(5.5, -.6), new
		// Translation2d(6.051, -1.524), new Translation2d(7, -.762),
		// new Translation2d(6.8, 0)), new Pose2d(0,0, new Rotation2d(Math.PI)),
		// DriveConstants.kTrajectoryConfig));

		// return new TrajectoryFollowCommand(m_driveSubsystem,
		// TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d()),
		// List.of(new Translation2d(2.7, 0), new Translation2d(3.6, -1.4), new
		// Translation2d(2.6, -1.6),
		// new Translation2d(1.7, -1.4), new Translation2d(2.5, 0.2), new
		// Translation2d(5.7, -.3),
		// new Translation2d(6, .9), new Translation2d(5.7, 1.1), new Translation2d(4.2,
		// .9), new Translation2d(4.3, 0),
		// new Translation2d(6.9, -1.7), new Translation2d(8.3, -1.1), new
		// Translation2d(8.2,0), new Translation2d(4,-0.1)),
		// new Pose2d(0,0, new Rotation2d()), DriveConstants.kTrajectoryConfig));

		return m_autoChooser.getSelected();
	}

	/**
	 * Generates all autonomous commands. UPDATED 2021: sets up trajectories to
	 * follow in AutoNav challenges and Galatic Search challenges
	 */
	private void generateAutonomousCommands() {

		// LOADING SLALOM PATH
		String trajectoryJSONSlalom = "/home/lvuser/deploy/Slalom.wpilib.json";
		Trajectory trajectorySlalom = new Trajectory();

		try {
			Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSONSlalom);
			trajectorySlalom = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
			Transform2d transform = new Pose2d(0, 0, new Rotation2d()).minus(trajectorySlalom.getInitialPose());
			trajectorySlalom = trajectorySlalom.transformBy(transform);
			m_autoChooser.addOption("Slalom", new TrajectoryFollowCommand(m_driveSubsystem, trajectorySlalom));
		} catch (IOException ex) {
			DriverStation.reportError("Unable to open trajectory: " + trajectoryJSONSlalom, ex.getStackTrace());
		}

		// LOADING ALL FOUR GALACTIC PATHS AND FEEDING THEM INTO THE COMMAND

		// PATH A BLUE
		String trajectoryJSONABlue = "/home/lvuser/deploy/GalacticSearchABlue.wpilib.json";
		Trajectory trajectoryABlue = new Trajectory();

		try {
			Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSONABlue);
			trajectoryABlue = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
			Transform2d transform = new Pose2d(0, 0, new Rotation2d()).minus(trajectoryABlue.getInitialPose());
			trajectoryABlue = trajectoryABlue.transformBy(transform);
		} catch (IOException ex) {
			DriverStation.reportError("Unable to open trajectory: " + trajectoryJSONABlue, ex.getStackTrace());
		}

		// PATH A RED
		String trajectoryJSONARed = "/home/lvuser/deploy/GalacticSearchARed.wpilib.json";
		Trajectory trajectoryARed = new Trajectory();

		try {
			Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSONARed);
			trajectoryARed = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
			Transform2d transform = new Pose2d(0, 0, new Rotation2d()).minus(trajectoryARed.getInitialPose());
			trajectoryARed = trajectoryARed.transformBy(transform);
		} catch (IOException ex) {
			DriverStation.reportError("Unable to open trajectory: " + trajectoryJSONARed, ex.getStackTrace());
		}

		// PATH B BLUE
		String trajectoryJSONBBlue = "/home/lvuser/deploy/GalacticSearchBBlue.wpilib.json";
		Trajectory trajectoryBBlue = new Trajectory();

		try {
			Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSONBBlue);
			trajectoryBBlue = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
			Transform2d transform = new Pose2d(0, 0, new Rotation2d()).minus(trajectoryBBlue.getInitialPose());
			trajectoryBBlue = trajectoryBBlue.transformBy(transform);
		} catch (IOException ex) {
			DriverStation.reportError("Unable to open trajectory: " + trajectoryJSONBBlue, ex.getStackTrace());
		}

		// PATH B RED
		String trajectoryJSONBRed = "/home/lvuser/deploy/GalacticSearchBRed.wpilib.json";
		Trajectory trajectoryBRed = new Trajectory();

		try {
			Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSONBRed);
			trajectoryBRed = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
			Transform2d transform = new Pose2d(0, 0, new Rotation2d()).minus(trajectoryBRed.getInitialPose());
			trajectoryBRed = trajectoryBRed.transformBy(transform);
		} catch (IOException ex) {
			DriverStation.reportError("Unable to open trajectory: " + trajectoryJSONBRed, ex.getStackTrace());
		}

		m_autoChooser.addOption("Galactic Search",
				new PixyGalacticCommand(m_arduinoSubsystem, m_driveSubsystem, trajectoryARed, trajectoryBRed,
						trajectoryABlue, trajectoryBBlue).deadlineWith(new IntakeCommand(m_intakeSubsystem),
								new RunCarouselCommand(m_carouselSubsystem, CarouselConstants.kIntakeVelocity),
								new BounceArmCommand(m_armSubsystem)));

		// CREATING BOUNCE PATH
		DriveConstants.kTrajectoryConfigREVERSED.setReversed(true);

		Command b1 = new TrajectoryFollowCommand(m_driveSubsystem,
				TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d()),
						List.of(new Translation2d(1, 0)), new Pose2d(1.05, 1.1, new Rotation2d(Math.PI / 2)),
						DriveConstants.kTrajectoryConfig));

		Command b2 = new TrajectoryFollowCommand(m_driveSubsystem, // this part needs to be updated to take out the
																	// weird zig-zag motion
				TrajectoryGenerator.generateTrajectory(new Pose2d(1.05, 1.1, new Rotation2d(Math.PI)), // so to prevent
																										// the zig-zag,
																										// shift 1 to
																										// 1.05
						List.of(new Translation2d(1.05, -.09), new Translation2d(1.8, -0.09),
								new Translation2d(1.8, -1), // then shift 1 to 1.05 or 1.10
								new Translation2d(3.55, -1)),
						new Pose2d(3.6, 1.4, new Rotation2d(-Math.PI / 2)), DriveConstants.kTrajectoryConfigREVERSED));

		Command b3 = new TrajectoryFollowCommand(m_driveSubsystem,
				TrajectoryGenerator.generateTrajectory(new Pose2d(3.6, 1.4, new Rotation2d(-Math.PI / 2)),
						List.of(new Translation2d(4.1, -.9), new Translation2d(5.8, -.9)),
						new Pose2d(6.2, 1.7, new Rotation2d(Math.PI / 2)), DriveConstants.kTrajectoryConfig));

		Command b4 = new TrajectoryFollowCommand(m_driveSubsystem,
				TrajectoryGenerator.generateTrajectory(new Pose2d(6.2, 1.7, new Rotation2d(Math.PI / 2)),
						List.of(new Translation2d(6.2, 1)), new Pose2d(8, 1, new Rotation2d(Math.PI)),
						DriveConstants.kTrajectoryConfigREVERSED));

		m_autoChooser.addOption("Bounce", b1.andThen(b2).andThen(b3).andThen(b4));

		// So try this first...
		// Command br = new TrajectoryFollowCommand(m_driveSubsystem,
		// TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d()),
		// List.of(
		// new Translation2d(2.54, 0), new Translation2d(3.302, -0.254), new
		// Translation2d(3.556, -0.635), new Translation2d(3.429, -1.27),
		// new Translation2d(2.794, -1.524), new Translation2d(2.032, -1.27), new
		// Translation2d(2.032, -0.508), new Translation2d(2.286, -0.254))
		// , new Pose2d(3.048, 0, new Rotation2d()), DriveConstants.kTrajectoryConfig));

		// Command br1 = new TrajectoryFollowCommand(m_driveSubsystem,
		// TrajectoryGenerator.generateTrajectory(new Pose2d(3.048, 0, new
		// Rotation2d()),
		// List.of(new Translation2d(5.08, -0.254), new Translation2d(5.588, 0.127), new
		// Translation2d(5.969, 0.508),
		// new Translation2d(5.842, 1.27), new Translation2d(5.08, 1.778), new
		// Translation2d(4.191, 1.143), new Translation2d(4.191, 0.635),
		// new Translation2d(4.699, -0.254), new Translation2d(5.334, -1.143), new
		// Translation2d(5.969, -1.778), new Translation2d(6.604, -1.905),
		// new Translation2d(7.366, -1.524), new Translation2d(7.62, -0.889), new
		// Translation2d(7.366, -0.254), new Translation2d(7.112, -0.0762),
		// new Translation2d(6.604, 0)), new Pose2d(0, 0, new Rotation2d(Math.PI)),
		// DriveConstants.kTrajectoryConfig));

		// Then try this...

		// CREATING BARREL RACING PATH
		Command barrelRacing = new TrajectoryFollowCommand(m_driveSubsystem, TrajectoryGenerator.generateTrajectory(
				new Pose2d(0, 0, new Rotation2d()),
				List.of(new Translation2d(2.8, 0.1), new Translation2d(3.3, -1), new Translation2d(2.45, -1.7),
						new Translation2d(1.56, -1), new Translation2d(2.386, 0.1), new Translation2d(5.1, -.1), 
						new Translation2d(5.75, 1.3), new Translation2d(4.8, 1.524), new Translation2d(3.9, .762),
						new Translation2d(4.527, 0), new Translation2d(6.6, -1.85), new Translation2d(7.1, -.762), //something about this ending was wrong
						new Translation2d(7.1, -.15)),
				new Pose2d(0, 0, new Rotation2d(Math.PI)), DriveConstants.kTrajectoryConfig));

		// m_autoChooser.addOption("Barrel racing", br.andThen(br1)); //based on data
		// from Andrew's spline program, TODO make my own --> like with the bounce path
		// --> make like a 45 degree angle for last turn?
		m_autoChooser.addOption("Barrel Racing", barrelRacing);

		// SOMETHING TO TEST OUT: BARREL RACING BASED ON PATHWEAVER
		// String trajectoryJSONBarrel = "/home/lvuser/deploy/BarrelRacing.wpilib.json";
		// Trajectory trajectoryBarrel = new Trajectory();

		// try {
		// 	Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSONBarrel);
		// 	trajectoryBarrel = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
		// 	Transform2d transform = new Pose2d(0, 0, new Rotation2d()).minus(trajectoryBarrel.getInitialPose());
		// 	trajectoryBarrel = trajectoryBarrel.transformBy(transform);
		// } catch (IOException ex) {
		// 	DriverStation.reportError("Unable to open trajectory: " + trajectoryJSONBarrel, ex.getStackTrace());
		// }

		// m_autoChooser.addOption("Galactic Search",
		// 		new PixyGalacticCommand(m_arduinoSubsystem, m_driveSubsystem, trajectoryARed, trajectoryBRed,
		// 				trajectoryABlue, trajectoryBBlue).deadlineWith(new IntakeCommand(m_intakeSubsystem),
		// 						new RunCarouselCommand(m_carouselSubsystem, CarouselConstants.kIntakeVelocity),
		// 						new BounceArmCommand(m_armSubsystem)));

	}
}