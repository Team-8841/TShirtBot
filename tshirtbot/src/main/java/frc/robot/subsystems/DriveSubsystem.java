package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
    
    // Define Motor controllers

    private final CANSparkMax m_leftFrontMotor = new CANSparkMax(DriveConstants.k_leftFrontMotorPort, MotorType.kBrushed);
    private final CANSparkMax m_leftBackMotor = new CANSparkMax(DriveConstants.k_leftBackMotorPort, MotorType.kBrushed);
    private final CANSparkMax m_rightFrontMotor = new CANSparkMax(DriveConstants.k_rightFrontMotorPort, MotorType.kBrushed);
    private final CANSparkMax m_rightBackMotor = new CANSparkMax(DriveConstants.k_rightBackMotorPort, MotorType.kBrushed);


    // Left side drivetrain
    private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(m_leftFrontMotor, m_leftBackMotor);

    // Right side drivetrain
    private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(m_rightFrontMotor, m_rightBackMotor);

    // Define Drive system
    private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);


    // Subsystem
    public DriveSubsystem() {
        configureSparkBrake(m_leftFrontMotor);
        configureSparkBrake(m_leftBackMotor);
        configureSparkBrake(m_rightFrontMotor);
        configureSparkBrake(m_rightBackMotor);

        m_drive.setMaxOutput(DriveConstants.k_MaxOutput);
    }

    private void configureSparkCoast(CANSparkMax sparkMax) {
        sparkMax.restoreFactoryDefaults();
        sparkMax.setOpenLoopRampRate(DriveConstants.k_DTRampRate);
        sparkMax.setSmartCurrentLimit(DriveConstants.k_CurrentLimit);
        sparkMax.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    private void configureSparkBrake(CANSparkMax sparkMax) {
        sparkMax.restoreFactoryDefaults();
        sparkMax.setOpenLoopRampRate(DriveConstants.k_DTRampRate);
        sparkMax.setSmartCurrentLimit(DriveConstants.k_CurrentLimit);
        sparkMax.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }


    public void RobotDrive(double left, double right){

        m_leftFrontMotor.setInverted(false);
        m_leftBackMotor.setInverted(false);
        m_rightFrontMotor.setInverted(true);
        m_rightBackMotor.setInverted(true);

        m_drive.tankDrive(left, right);
    }

}
