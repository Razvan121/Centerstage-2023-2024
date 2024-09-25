package org.firstinspires.ftc.teamcode.NEDRobot.Subsystems;

import org.firstinspires.ftc.teamcode.NEDRobot.Hardware.NEDSubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.util.MathUtil;

public class LiftSubsystem extends NEDSubsystem {
    private final Obot obot = Obot.getInstance();

    public enum WristState{
        HORIZONTAL_TRANSFER,
        HORIZONTAL_DEPOSIT,
        VERTICAL,
        OBLIC_LEFT,
        OBLIC_RIGHT
    }
    public enum PitchState{
        HOME,
        TRANSFER,
        VERTICAL,
        LOW,
        MID,
        HIGH,
        SEMITRANSFER,
        IN
    }
    public enum LiftState{
        HOME,
        LOW,
        MID,
        HIGH,
        LOW_TELEOP
    }
    public enum BucketState{
        HOME,
        TRANSFER,
        LOW,
        MID,
        HIGH,
        IN,
        SEMITRANSFER,
        SEMIOUT,
        SEMIIN,
        HANG
    }
    public enum TriggerState{
        OPEN,
        CLOSE,
        TRANSFER
    }
    public enum LinkageState{
        HOME,
        EXTEND_LEFT,
        EXTEND_RIGHT,
        TRANSFER
    }
    public enum TurretState{
        AUTO_LEFT,
        AUTO_RIGHT,
        OUT
    }
    public enum AngleState{
        AUTO_LEFT,
        AUTO_RIGHT,
        OUT
    }
    public enum AirplaneState {
        HOME,
        RELEASE,
    }

    private WristState wristState;
    private PitchState pitchState;
    private LinkageState linkageState;
    private LiftState liftState;
    private BucketState bucketState;
    private TriggerState triggerState;
    private AngleState angleState;
    private TurretState turretState;
    private AirplaneState airplaneState;
    private int backdropHeight = 0;


    private double HorizontalTransferWristPos=0.225;
    private double HorizontalDepositWristPos=0.79;
    private double OblicLeftWristPos=0.14;
    private double OblicRightWristPos=0.14;
    private double VerticalWristPos;


    private double HomePitchPos=0.83;//72 -- 59
    private double TransferPitchPos=0.43;//67
    private double VerticalPitchPos=0.58;//32
    private double LowPitchPos=0.05;//34
    private double MidPitchPos=0.05;//34
    private double HighPitchPos=0.05;//34
    private double SemitransferPitchPos=0.57;
    private double InPitchPos=0.55;
    private double[] BackdropHeights = {
            -475,
            -600,
            -800,
            -1000,
            -1200,
            -1400,
            -1600,
            -1800,
            -2100,
    };
    private double[] PitchBackdropHeights = {
            0.05,
            0.05,
            0.05,
            0.05,
            0.05,
            0.05,
            0.05,
            0.05,
            0.05,
            0.05,
            0.05,
            0.05,
            0.05,
            0.05,
            0.05
    };
    private double LIFT_MANUAL_FACTOR=10;
    private double HomeLiftPos=0;
    private double LowLiftPos=-900;
    private double MidLiftPos=-1700;
    private double HighLiftPos=-2100;
    private double LowTeleOpLiftPos=-300;


    private double HomeBucketPos=63;
    private double TransferBucketPos=50;
    private double LowBucketPos=260;
    private double MidBucketPos=260;
    private double HighBucketPos=260;
    private double InBucketPos=105;
    private double SemitransferBucketPos=56;
    private double SemiOutBucketPos=140;
    private double SemiInBucketPos=70;
    private double HangBucketPos=200;

    private double LeftOpenTriggerPos=0.2;
    private double RightOpenTriggerPos=0.9;
    private double RightCloseTriggerPos=0.3;
    private double LeftCloseTriggerPos=0.92;
    private double LeftTransferTriggerPos=0.45;
    private double RightTransferTriggerPos=0.65;

    private double LinkageExtendLeftPos=0.1;
    private double LinkageExtendRightPos=0.1;
    private double LinkageHomePos=0.35;
    private double LinkageTransferPos=0.38;
    private double LinkageInPos;

    private double AngleAutoLeftPos=0;
    private double AngleAutoRightPos=0;
    private double AngleOutPos =0.335;

    private double TurretAutoLeftPos=0;
    private double TurretAutoRightPos=0;
    private double TurretOutPos=0.425;

    double HomePos=0.75;
    double ReleasePos=0;


    public LiftSubsystem()
    {

    }
    @Override
    public void periodic() {
        obot.Lift.periodic();
    }

    @Override
    public void read() {
        obot.Lift.read();
    }

    @Override
    public void write() {
        obot.Lift.write();
    }

    @Override
    public void reset() {

    }

    public void update(AngleState state)
    {
        angleState=state;
        switch (angleState)
        {
            case AUTO_LEFT:
                obot.angle.setPosition(AngleAutoLeftPos);
                break;
            case AUTO_RIGHT:
                obot.angle.setPosition(AngleAutoRightPos);
                break;
            case OUT:
                obot.angle.setPosition(AngleOutPos);
                break;
        }
    }
    public void update(PitchState state)
    {
        pitchState=state;
        switch(pitchState)
        {
            case HOME:
                obot.PitchOuttake.setPosition(HomePitchPos);
                break;
            case TRANSFER:
                obot.PitchOuttake.setPosition(TransferPitchPos);
                break;
            case LOW:
                obot.PitchOuttake.setPosition(LowPitchPos);
                break;
            case MID:
                obot.PitchOuttake.setPosition(MidPitchPos);
                break;
            case HIGH:
                obot.PitchOuttake.setPosition(HighPitchPos);
                break;
            case VERTICAL:
                obot.PitchOuttake.setPosition(VerticalPitchPos);
                break;
            case SEMITRANSFER:
                obot.PitchOuttake.setPosition(SemitransferPitchPos);
                break;
            case IN:
                obot.PitchOuttake.setPosition(InPitchPos);
                break;
        }
    }

    public void update(WristState state)
    {
        wristState=state;
        switch(wristState)
        {
            case HORIZONTAL_TRANSFER:
                obot.WristOuttake.setPosition(HorizontalTransferWristPos);
                break;
            case HORIZONTAL_DEPOSIT:
                obot.WristOuttake.setPosition(HorizontalDepositWristPos);
            case OBLIC_LEFT:
                obot.WristOuttake.setPosition(OblicLeftWristPos);
                break;
            case OBLIC_RIGHT:
                obot.WristOuttake.setPosition(OblicRightWristPos);
                break;
            case VERTICAL:
                obot.WristOuttake.setPosition(VerticalWristPos);
                break;
        }
    }

    public void update(BucketState state)
    {
        bucketState=state;
        switch(bucketState)
        {
            case HOME:
                obot.leftBucket.setPosition(HomeBucketPos/360);
                obot.rightBucket.setPosition(HomeBucketPos/360);
                break;
            case TRANSFER:
                obot.leftBucket.setPosition(TransferBucketPos/360);
                obot.rightBucket.setPosition(TransferBucketPos/360);
                break;
            case LOW:
                obot.leftBucket.setPosition(LowBucketPos/360);
                obot.rightBucket.setPosition(LowBucketPos/360);
                break;
            case MID:
                obot.leftBucket.setPosition(MidBucketPos/360);
                obot.rightBucket.setPosition(MidBucketPos/360);
                break;
            case HIGH:
                obot.leftBucket.setPosition(HighBucketPos/360);
                obot.rightBucket.setPosition(HighBucketPos/360);
                break;
            case IN:
                obot.leftBucket.setPosition(InBucketPos/360);
                obot.rightBucket.setPosition(InBucketPos/360);
                break;
            case SEMITRANSFER:
                obot.leftBucket.setPosition(SemitransferBucketPos/360);
                obot.rightBucket.setPosition(SemitransferBucketPos/360);
                break;
            case SEMIOUT:
                obot.leftBucket.setPosition(SemiOutBucketPos/360);
                obot.rightBucket.setPosition(SemiOutBucketPos/360);
                break;
            case SEMIIN:
                obot.leftBucket.setPosition(SemiInBucketPos/360);
                obot.rightBucket.setPosition(SemiInBucketPos/360);
                break;
            case HANG:
                obot.leftBucket.setPosition(HangBucketPos/360);
                obot.rightBucket.setPosition(HangBucketPos/360);
                break;

        }
    }

    public void update(TriggerState state)
    {
        triggerState=state;
        switch (triggerState){
            case OPEN:
                obot.leftTrigger.setPosition(LeftOpenTriggerPos);
                obot.rightTrigger.setPosition(RightOpenTriggerPos);
                break;
            case CLOSE:
                obot.leftTrigger.setPosition(LeftCloseTriggerPos);
                obot.rightTrigger.setPosition(RightCloseTriggerPos);
                break;
            case TRANSFER:
                obot.leftTrigger.setPosition(LeftTransferTriggerPos);
                obot.rightTrigger.setPosition(RightTransferTriggerPos);
                break;
        }
    }

    public void update(LiftState state)
    {
        liftState=state;
        switch(liftState)
        {
            case HOME:
                obot.Lift.setMotionProfileTargetPosition(HomeLiftPos);
                break;
            case LOW:
                obot.Lift.setMotionProfileTargetPosition(LowLiftPos);
                break;
            case MID:
                obot.Lift.setMotionProfileTargetPosition(MidLiftPos);
                break;
            case HIGH:
                obot.Lift.setMotionProfileTargetPosition(HighLiftPos);
                break;
            case LOW_TELEOP:
                obot.Lift.setMotionProfileTargetPosition(LowTeleOpLiftPos);
        }
    }

    public void update(TurretState state){
        turretState=state;
        switch (turretState){
            case OUT:
                obot.turret.setPosition(TurretOutPos);
                break;
            case AUTO_LEFT:
                obot.turret.setPosition(TurretAutoLeftPos);
                break;
            case AUTO_RIGHT:
                obot.turret.setPosition(TurretAutoRightPos);
        }
    }

    public void update(LinkageState state){
        linkageState=state;
        switch (linkageState){
            case HOME:
                obot.linkage.setPosition(LinkageHomePos);
                break;
            case EXTEND_LEFT:
                obot.linkage.setPosition(LinkageExtendLeftPos);
                break;
            case EXTEND_RIGHT:
                obot.linkage.setPosition(LinkageExtendRightPos);
                break;
            case TRANSFER:
                obot.linkage.setPosition(LinkageTransferPos);
        }
    }
    public void update(AirplaneState state)
    {
        state = airplaneState;
        switch (state){
            case HOME:
                obot.airplane.setPosition(HomePos);
                break;
            case RELEASE:
                obot.airplane.setPosition(ReleasePos);
        }
    }


    public int getBackdropHeightIndex() {
        return backdropHeight;
    }
    public double getBackdropHeight() {
        return BackdropHeights[getBackdropHeightIndex()];
    }
    public double getPitchBackdropHeight() {
        return PitchBackdropHeights[getBackdropHeightIndex()];
    }

    public void incrementBackdropHeight(int amount) {
        this.backdropHeight = (int) MathUtil.clamp(getBackdropHeightIndex() + amount, 0, 15);
    }
    public void setBackdropHeight(int amount) {
        this.backdropHeight = (int) MathUtil.clamp(amount, 0, 15);
    }

}
