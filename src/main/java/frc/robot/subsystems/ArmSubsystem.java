package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
    private CANSparkMax extensionMotor = new CANSparkMax(ArmConstants.kExtensionMotor, MotorType.kBrushed);
    private CANSparkMax rightAngleMotor  = new CANSparkMax(ArmConstants.kAngleMotor, MotorType.kBrushed);
    private CANSparkMax leftAngleMotor  = new CANSparkMax(ArmConstants.kAngleMotor, MotorType.kBrushed);
    private SparkMaxPIDController m_extensionPidController;
    public SparkMaxPIDController m_rightAnglePidController;
    public SparkMaxPIDController m_leftAnglePidController;
    
    private SparkMaxAlternateEncoder m_extensionEncoder;
    private SparkMaxAlternateEncoder m_rightAngleEncoder;
    private SparkMaxAlternateEncoder m_leftAngleEncoder;

public ArmSubsystem() {

    m_extensionEncoder = new SparkMaxAlternateEncoder(extensionMotor,4096);
    m_leftAngleEncoder = leftAngleMotor.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 4096);
    m_rightAngleEncoder = rightAngleMotor.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 4096);

    m_extensionPidController = extensionMotor.getPIDController();
    m_leftAnglePidController = leftAngleMotor.getPIDController();
    m_rightAnglePidController = rightAngleMotor.getPIDController();

    m_extensionPidController.setFeedbackDevice(m_extensionEncoder);
    m_leftAnglePidController.setFeedbackDevice(m_leftAngleEncoder);
    m_rightAnglePidController.setFeedbackDevice(m_rightAngleEncoder);

    // set PID coefficients
    //m_extensionPidController.setP(kExtensionP);
    m_extensionPidController.setI(ArmConstants.kExtensionI);
    m_extensionPidController.setD(ArmConstants.kExtensionD);
    m_extensionPidController.setIZone(ArmConstants.kExtensionIz);
    m_extensionPidController.setFF(ArmConstants.kExtensionFF);
    m_extensionPidController.setOutputRange(ArmConstants.kExtensionMinOutput, ArmConstants.kExtensionMaxOutput);

    m_leftAnglePidController.setP(ArmConstants.kAngleP);
    m_leftAnglePidController.setI(ArmConstants.kAngleI);
    m_leftAnglePidController.setD(ArmConstants.kAngleD);
    m_leftAnglePidController.setIZone(ArmConstants.kAngleIz);
    m_leftAnglePidController.setFF(ArmConstants.kAngleFF);
    m_leftAnglePidController.setOutputRange(ArmConstants.kAngleMinOutput, ArmConstants.kAngleMaxOutput);

    m_rightAnglePidController.setP(ArmConstants.kAngleP);
    m_rightAnglePidController.setI(ArmConstants.kAngleI);
    m_rightAnglePidController.setD(ArmConstants.kAngleD);
    m_rightAnglePidController.setIZone(ArmConstants.kAngleIz);
    m_rightAnglePidController.setFF(ArmConstants.kAngleFF);
    m_rightAnglePidController.setOutputRange(ArmConstants.kAngleMinOutput, ArmConstants.kAngleMaxOutput);

}
    public void extendopatronum(double reach) {
        m_extensionPidController.setReference(reach,  CANSparkMax.ControlType.kPosition);


    }

    public void wingardiumleviosa(double reach) {
        m_rightAnglePidController.setReference(reach,  CANSparkMax.ControlType.kPosition);
        m_leftAnglePidController.setReference(reach,  CANSparkMax.ControlType.kPosition);
        

    }
}