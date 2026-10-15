package org.ftcsim.runner;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.nio.charset.StandardCharsets;

/**
 * Wire format shared with the simulator (sim-core/src/proto.rs).
 * Frames are {@code u32 length, u8 type, body}; big-endian; strings are {@code u16 length + UTF-8}.
 */
final class Protocol {
    static final int VERSION = 4;

    static final int HELLO = 1;
    static final int COMMAND = 2;
    static final int CONFIG = 10;
    static final int STATE = 11;
    static final int CONTROL = 12;

    static final int DEVICE_MOTOR = 0;
    static final int DEVICE_IMU = 1;
    static final int DEVICE_SERVO = 2;
    static final int DEVICE_CRSERVO = 3;
    static final int DEVICE_PINPOINT = 4;

    static final int CONTROL_INIT = 1;
    static final int CONTROL_START = 2;
    static final int CONTROL_STOP = 3;

    // Gamepad button bits, matching sim-core/src/gamepad.rs.
    static final int A = 1, B = 1 << 1, X = 1 << 2, Y = 1 << 3, LEFT_BUMPER = 1 << 4, RIGHT_BUMPER = 1 << 5;
    static final int DPAD_UP = 1 << 6, DPAD_DOWN = 1 << 7, DPAD_LEFT = 1 << 8, DPAD_RIGHT = 1 << 9;
    static final int START = 1 << 10, BACK = 1 << 11, GUIDE = 1 << 12, LEFT_STICK_BUTTON = 1 << 13, RIGHT_STICK_BUTTON = 1 << 14;

    private Protocol() {
    }

    static final class Frame {
        final int type;
        final DataInputStream body;

        Frame(int type, DataInputStream body) {
            this.type = type;
            this.body = body;
        }
    }

    static Frame read(DataInputStream in) throws IOException {
        int length = in.readInt();
        if (length <= 0 || length > (1 << 20)) throw new IOException("bad frame length " + length);
        byte[] buf = new byte[length];
        in.readFully(buf);
        return new Frame(buf[0] & 0xff, new DataInputStream(new ByteArrayInputStream(buf, 1, length - 1)));
    }

    /** Builds a message body; {@link #send} frames and flushes it. */
    static final class Message {
        final int type;
        final ByteArrayOutputStream bytes = new ByteArrayOutputStream();
        final DataOutputStream out = new DataOutputStream(bytes);

        Message(int type) {
            this.type = type;
        }

        void send(DataOutputStream to) throws IOException {
            out.flush();
            to.writeInt(bytes.size() + 1);
            to.writeByte(type);
            bytes.writeTo(to);
            to.flush();
        }
    }

    static void writeString(DataOutputStream out, String s) throws IOException {
        byte[] b = s.getBytes(StandardCharsets.UTF_8);
        int n = Math.min(b.length, 0xffff);
        out.writeShort(n);
        out.write(b, 0, n);
    }

    static String readString(DataInputStream in) throws IOException {
        byte[] b = new byte[in.readUnsignedShort()];
        in.readFully(b);
        return new String(b, StandardCharsets.UTF_8);
    }
}
