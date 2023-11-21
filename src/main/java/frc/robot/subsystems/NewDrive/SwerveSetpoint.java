package frc.robot.subsystems.NewDrive;

import frc.robot.Utils.Chessy.CheesyChassisSpeeds;

public class SwerveSetpoint {
    public CheesyChassisSpeeds mChassisSpeeds;
    public CheesySwerveModuleState[] mModuleStates;

    public SwerveSetpoint(CheesyChassisSpeeds chassisSpeeds, CheesySwerveModuleState[] initialStates) {
        this.mChassisSpeeds = chassisSpeeds;
        this.mModuleStates = initialStates;
    }

    @Override
    public String toString() {
        String ret = mChassisSpeeds.toString() + "\n";
        for (int i = 0; i < mModuleStates.length; ++i ) {
            ret += "  " + mModuleStates[i].toString() + "\n";
        }
        return ret;
    }
}
