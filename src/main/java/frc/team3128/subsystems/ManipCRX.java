package frc.team3128.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3128.common.hardware.motorcontroller.NAR_TalonSRX;
import frc.team3128.common.utility.NAR_Shuffleboard;

import static frc.team3128.Constants.CRXConstants.*;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class ManipCRX extends SubsystemBase {
    private NAR_TalonSRX m_roller;

    private static ManipCRX instance;

    private boolean outtaking = false;

    private boolean CONE = false;

    private ManipCRX() {
        configMotor();
    }

    public static synchronized ManipCRX getInstance() {
        if (instance == null) {
            instance = new ManipCRX();
        }
        return instance;
    }

    @Override
    public void periodic() {
        // if (hasObjectPresent() && !outtaking) {
        //     stopRoller();
        // }
    }

    public void configMotor() {
        m_roller = new NAR_TalonSRX(ROLLER_MOTOR_ID);
        m_roller.setNeutralMode(NeutralMode.Brake);
        m_roller.setInverted(true);
    }

    public void intake(boolean cone) {
        CONE = cone;
        outtaking = false;
        m_roller.set(cone ? CONE_ROLLER_POWER : CUBE_ROLLER_POWER);
    }

    public void outtake() {
        outtaking = true;
        m_roller.set(OUTTAKE_POWER);
    }

    public void shoot() {
        outtaking = true;
        m_roller.set(SHOOT_POWER);
    }

    public void stopRoller() {
        m_roller.set(0);
    }

    public boolean hasObjectPresent() {
        return (CONE ? Math.abs(m_roller.getStatorCurrent()) > CONE_CURRENT_THRESHOLD : Math.abs(m_roller.getStatorCurrent()) > CUBE_CURRENT_THRESHOLD);
    }
    public void initShuffleboard() {
        NAR_Shuffleboard.addData("Manipulator", "manip current", () -> m_roller.getStatorCurrent(), 0, 1);
        NAR_Shuffleboard.addData("Manipulator", "get", () -> m_roller.getMotorOutputPercent(), 0, 3);
        NAR_Shuffleboard.addData("Manipulator", "ObjectPresent", () -> hasObjectPresent(), 1, 1);
    }
}
