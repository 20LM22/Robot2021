package frc.robot.commands.drivecommands;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class LimelightTurnCommand extends CommandBase {

    private final LimelightSubsystem m_limelightSubsystem;
    private final DriveSubsystem m_driveSubsystem;
    private final double m_turnGoal;
    private final ProfiledPIDController m_turnController = new ProfiledPIDController(LimelightConstants.kTurnP,
            LimelightConstants.kTurnI, LimelightConstants.kTurnD, new Constraints(
                    DriveConstants.kMaxRotSpeedMetersPerSecond, DriveConstants.kMaxAccelerationMetersPerSecondSquared));

    /**
     * Use the limelight to reach a desired angle to the powerport
     * 
     * @param limelightSubsystem The limelight subsystem to gather data from
     * @param driveSubsystem     The drivetrain subsystem to be used
     * @param turnGoal           Supplier of the angle setpoint towards the target
     */
    public LimelightTurnCommand(LimelightSubsystem limelightSubsystem, DriveSubsystem driveSubsystem, double turnGoal) {
        m_limelightSubsystem = limelightSubsystem;
        m_driveSubsystem = driveSubsystem;
        m_turnGoal = turnGoal;
        addRequirements(m_driveSubsystem);
    }

    /**
     * Set the tolerance and goal of the PID
     */
    public void initialize() {
        m_limelightSubsystem.turnOnLight();
        m_turnController.setTolerance(LimelightConstants.kTurnTolerance);
        m_turnController.setGoal(m_turnGoal);
    }

    /**
     * Update the motor outputs
     */
    public void execute() {
        
        double robotTurnSpeed = m_turnController.calculate(m_limelightSubsystem.getXAngle()); //m_limelightSubsystem.getXAngle()
        DifferentialDriveWheelSpeeds wheelSpeeds = DriveConstants.kDriveKinematics
                .toWheelSpeeds(new ChassisSpeeds(0, 0, robotTurnSpeed));
        //m_driveSubsystem.setWheelSpeeds(wheelSpeeds);

        double leftVoltage = DriveConstants.kFeedForward.calculate(wheelSpeeds.leftMetersPerSecond);
        double rightVoltage = DriveConstants.kFeedForward.calculate(wheelSpeeds.rightMetersPerSecond);
        m_driveSubsystem.tankDriveVolts(leftVoltage, rightVoltage);
        //TODO if tuning the pid based on just the talons doesn't work, could write a separate method to handle voltages with spark maxes
    }

    /**
     * Stop the drivetrain at the end of the command
     */
    public void end(boolean interrputed) {
        m_limelightSubsystem.turnOffLight();
        m_driveSubsystem.tankDrive(0, 0);
    }

}

// float Kp = -0.1f; //try this?
// float min_command = 0.05f;

// std::shared_ptr<NetworkTable> table = NetworkTable::GetTable("limelight");
// float tx = table->GetNumber("tx");

// if (joystick->GetRawButton(9))
// {
//         float heading_error = -tx;
//         float steering_adjust = 0.0f;
//         if (tx > 1.0)
//         {
//                 steering_adjust = Kp*heading_error - min_command;
//         }
//         else if (tx < 1.0)
//         {
//                 steering_adjust = Kp*heading_error + min_command;
//         }
//         left_command += steering_adjust;
//         right_command -= steering_adjust;
// }