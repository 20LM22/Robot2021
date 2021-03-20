package frc.robot.commands.drivecommands;

import frc.robot.Constants.CarouselConstants;
import frc.robot.Constants.ArduinoConstants.ShooterLEDModes;
import frc.robot.commands.arduinocommands.UpdateLEDsCommand;
import frc.robot.commands.armcommands.BounceArmCommand;
import frc.robot.commands.carouselcommands.RunCarouselCommand;
import frc.robot.commands.intakecommands.IntakeCommand;
import frc.robot.subsystems.ArduinoSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CarouselSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArduinoConstants.LEDColorValues;
import frc.robot.Constants.ArduinoConstants.MainLEDModes;

public class PixyGalacticCommand extends SequentialCommandGroup {

    private final ArduinoSubsystem m_arduinoSubsystem;
    private final DriveSubsystem m_driveSubsystem;

    private final Trajectory m_trajectory;

    private final Trajectory m_redA;
    private final Trajectory m_redB;
    private final Trajectory m_blueA;
    private final Trajectory m_blueB;

    private final double aredx = 140; // TODO MUST TUNE THESE
    private final double bredx = 260;
    private final double bbluex = 85;
    // double bbluex=100; //potentially not needed because it will be the "else" do
    // this one to avoid confusion
    private final double threshhold = 20;

    public PixyGalacticCommand(ArduinoSubsystem arduinoSubsystem, DriveSubsystem driveSubsystem,
            Trajectory redA, Trajectory redB, Trajectory blueA, Trajectory blueB) {

        m_arduinoSubsystem = arduinoSubsystem;
        m_driveSubsystem = driveSubsystem;

        m_redA = redA;
        m_redB = redB;
        m_blueA = blueA;
        m_blueB = blueB;

        m_arduinoSubsystem.update();

        double color;

        if (m_arduinoSubsystem.getXValue() >= aredx - threshhold && m_arduinoSubsystem.getXValue() <= aredx + threshhold) {
            m_trajectory = m_redA;
            System.out.println("running red a");
            color = LEDColorValues.kRed;
        } else if (m_arduinoSubsystem.getXValue() >= bredx - threshhold && m_arduinoSubsystem.getXValue() <= bredx + threshhold) {
            System.out.println("running red b");
            m_trajectory = m_redB;
            color = LEDColorValues.kBlue;
        } else if (m_arduinoSubsystem.getXValue() >= bbluex - threshhold && m_arduinoSubsystem.getXValue() <= bbluex + threshhold) {
            System.out.println("running blue b");
            m_trajectory = m_blueB;
            color = LEDColorValues.kRed;
        } else {
            System.out.println("running blue a");
            m_trajectory = m_blueA;
            color = LEDColorValues.kBlue;
        }

        addRequirements(m_driveSubsystem, m_arduinoSubsystem);
        addCommands(new TrajectoryFollowCommand(m_driveSubsystem, m_trajectory));

        // new UpdateLEDsCommand(m_arduinoSubsystem, () -> {
		// 	return MainLEDModes.kFlashing;
		// }, () -> {
		// 	return color;
		// }, () -> {
		// 	return ShooterLEDModes.kFlashing;
		// }, () -> {
		// 	return color;
		// }));

    }

    public void end() {
        m_driveSubsystem.tankDrive(0, 0);
    }

}