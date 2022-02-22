// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpiutil.math.MathUtil;

/** Add your docs here. */
public class LinearServo extends Servo {
    double m_speed;
    double m_length;

    double setPos;
    double curPos;
    public LinearServo(int channel, int length, int speed) {
        super(channel);
        setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
        m_length = length; //Enters Length of Channel HERE mm
        m_speed = speed; //Enters Speed of Channel HERE mm/s (Max seen at actuonix.com 35:1)
    }

    public void setPosition(double setpoint) {
        setPos = MathUtil.clamp(setpoint, 0, m_length);
        setSpeed((setPos/m_length*2)-1);
    }
    double lastTime = 0;

    public void updateCurPos() {
        double dt = Timer.getFPGATimestamp() - lastTime;
        if (curPos > setPos + m_speed * dt) {
            curPos -= m_speed * dt;
        } else if(curPos < setPos - m_speed * dt) {
            curPos += m_speed * dt;
        } else {
            curPos = setPos;
        }
    }

    public double getPosition() {
        return curPos;
    }

    public boolean isFinished(){
        return curPos == setPos;
    }
}
