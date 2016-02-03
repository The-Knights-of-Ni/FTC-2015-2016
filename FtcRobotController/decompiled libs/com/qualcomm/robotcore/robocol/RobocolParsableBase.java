/*
 * Decompiled with CFR 0_101.
 */
package com.qualcomm.robotcore.robocol;

import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.robocol.RobocolParsable;
import com.qualcomm.robotcore.util.TypeConversion;
import java.nio.ByteBuffer;
import java.util.concurrent.atomic.AtomicInteger;

public abstract class RobocolParsableBase
implements RobocolParsable {
    protected int sequenceNumber;
    protected long nanotimeTransmit;
    protected static final long nanotimeTransmitInterval = 200000000;
    protected static AtomicInteger nextSequenceNumber = new AtomicInteger();

    public static void initializeSequenceNumber(int sequenceNumber) {
        nextSequenceNumber = new AtomicInteger(sequenceNumber);
    }

    public RobocolParsableBase() {
        this.setSequenceNumber();
        this.nanotimeTransmit = 0;
    }

    @Override
    public int getSequenceNumber() {
        return this.sequenceNumber;
    }

    public void setSequenceNumber(short sequenceNumber) {
        this.sequenceNumber = TypeConversion.unsignedShortToInt(sequenceNumber);
    }

    @Override
    public void setSequenceNumber() {
        this.setSequenceNumber((short)nextSequenceNumber.getAndIncrement());
    }

    @Override
    public byte[] toByteArrayForTransmission() throws RobotCoreException {
        byte[] arrby = this.toByteArray();
        this.nanotimeTransmit = System.nanoTime();
        return arrby;
    }

    @Override
    public boolean shouldTransmit(long nanotimeNow) {
        return this.nanotimeTransmit == 0 || nanotimeNow - this.nanotimeTransmit > 200000000;
    }

    protected ByteBuffer allocateWholeWriteBuffer(int overallSize) {
        return ByteBuffer.allocate(overallSize);
    }

    protected ByteBuffer getWholeReadBuffer(byte[] byteArray) {
        return ByteBuffer.wrap(byteArray);
    }

    protected ByteBuffer getWriteBuffer(int payloadSize) {
        ByteBuffer byteBuffer = this.allocateWholeWriteBuffer(5 + payloadSize);
        byteBuffer.put(this.getRobocolMsgType().asByte());
        byteBuffer.putShort((short)payloadSize);
        byteBuffer.putShort((short)this.sequenceNumber);
        return byteBuffer;
    }

    protected ByteBuffer getReadBuffer(byte[] byteArray) {
        int n = 3;
        ByteBuffer byteBuffer = ByteBuffer.wrap(byteArray, n, byteArray.length - n);
        this.setSequenceNumber(byteBuffer.getShort());
        return byteBuffer;
    }
}

