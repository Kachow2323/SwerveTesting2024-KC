package frc.robot;

public final class States {
    public enum ArmPos {
        STOW(0), SCORE(1);
        int val;
        private ArmPos(int val) {
            this.val = val;
        }
    }

    public enum HookPos {
        STOW(0), OPEN(1);//, SCORE(2);
        int val;
        private HookPos(int val) {
            this.val = val;
        }
    }
}
// Class to refer to all arm and hook states