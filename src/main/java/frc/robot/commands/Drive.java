package frc.robot.commands;


import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Globals;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;

/** An example command that uses an example subsystem. */
public class Drive extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    //private final Drivetrain m_subsystem;

    double x;
    double y;
    double rot;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public Drive(Drivetrain subsystem) {
        //m_subsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(Drivetrain.getInstance());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        /**
         * Note: Front in Teleop is different from "official" front!
         * */

        if (!Globals.isAiming) {
            x = Robot.oi.driver.getLeftY();
            y = -Robot.oi.driver.getLeftX();
            rot = Robot.oi.driver.getRightX();

            x = Math.abs(x) > 0.07 ? x : 0;
            y = Math.abs(y) > 0.07 ? y : 0;
            rot = Math.abs(rot) > 0.07 ? rot : 0;

            //System.out.println(x + "  ,  " + y);
            // A split-stick arcade command, with forward/backward controlled by the left
            // hand, and turning controlled by the right.
            Drivetrain.drive(
                    6.0 * x, //was 6
                    6.0 * y,
                    4.0 * rot,
                    false);
    /*Drivetrain.getInstance().setSpeedRR(
            -Robot.oi.driver.getY(GenericHID.Hand.kLeft),
            -Robot.oi.driver.getY(GenericHID.Hand.kRight)
    );
    Drivetrain.getInstance().setSpeedFL(
            -Robot.oi.driver.getY(GenericHID.Hand.kLeft),
            -Robot.oi.driver.getY(GenericHID.Hand.kRight)
    );
    System.out.println("INPUT: " + -Robot.oi.driver.getY(GenericHID.Hand.kRight));*/
            //System.out.println(rot);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
