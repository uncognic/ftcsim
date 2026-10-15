package org.ftcsim.runner;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.ConnectException;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.util.List;
import org.ftcsim.sdk.SimBus;

/**
 * The simulated Robot Controller. Finds the OpModes in TeamCode, connects to the simulator,
 * and runs whichever OpMode the Driver Station (the simulator window) selects.
 */
public final class Main {
    /** How long to wait for the OpMode to finish a tick before answering anyway (a loop with no hardware calls). */
    private static final long PARK_TIMEOUT_MS = 50;

    private Main() {
    }

    public static void main(String[] args) throws InterruptedException {
        String host = "127.0.0.1";
        int port = 7171;
        for (int i = 0; i < args.length - 1; i++) {
            if (args[i].equals("--host")) host = args[++i];
            else if (args[i].equals("--port")) port = Integer.parseInt(args[++i]);
        }

        List<OpModeScanner.Entry> opModes = OpModeScanner.scan();
        System.out.println("[runner] found " + opModes.size() + " OpModes:");
        for (OpModeScanner.Entry e : opModes) {
            System.out.println("  " + (e.autonomous ? "Autonomous" : "TeleOp    ") + "  " + e.name + "  (" + e.type.getName() + ")");
        }

        boolean announced = false;
        while (true) {
            try (Socket socket = new Socket()) {
                socket.connect(new InetSocketAddress(host, port), 1000);
                socket.setTcpNoDelay(true);
                System.out.println("[runner] connected to simulator at " + host + ":" + port);
                announced = false;
                runSession(socket, opModes);
            } catch (ConnectException e) {
                if (!announced) System.out.println("[runner] waiting for the simulator on " + host + ":" + port + " ...");
                announced = true;
            } catch (IOException e) {
                System.out.println("[runner] simulator disconnected: " + e.getMessage());
            }
            Thread.sleep(1000);
        }
    }

    private static void runSession(Socket socket, List<OpModeScanner.Entry> opModes) throws IOException {
        DataInputStream in = new DataInputStream(new BufferedInputStream(socket.getInputStream()));
        DataOutputStream out = new DataOutputStream(new BufferedOutputStream(socket.getOutputStream()));

        Protocol.Message hello = new Protocol.Message(Protocol.HELLO);
        hello.out.writeShort(Protocol.VERSION);
        hello.out.writeShort(opModes.size());
        for (OpModeScanner.Entry e : opModes) {
            Protocol.writeString(hello.out, e.name);
            hello.out.writeByte(e.autonomous ? 0 : 1);
            Protocol.writeString(hello.out, e.group);
        }
        hello.send(out);

        Protocol.Frame config = Protocol.read(in);
        if (config.type != Protocol.CONFIG) throw new IOException("expected CONFIG, got message type " + config.type);
        SimHardware hardware = new SimHardware(config.body);
        System.out.println("[runner] robot \"" + hardware.robotName + "\" with " + hardware.hardwareMap.size() + " devices");
        OpModeManager manager = new OpModeManager(opModes, hardware);
        long overruns = 0;

        try {
            while (true) {
                Protocol.Frame frame = Protocol.read(in);
                if (frame.type == Protocol.CONTROL) {
                    int action = frame.body.readUnsignedByte();
                    int index = frame.body.readUnsignedShort();
                    if (action == Protocol.CONTROL_INIT) manager.init(index);
                    else if (action == Protocol.CONTROL_START) manager.start();
                    else if (action == Protocol.CONTROL_STOP) manager.stop();
                } else if (frame.type == Protocol.STATE) {
                    int seq = frame.body.readInt();
                    double time = frame.body.readDouble();
                    hardware.applyState(frame.body);
                    SimBus.onTick(time);
                    if (!SimBus.awaitParked(PARK_TIMEOUT_MS) && ++overruns % 200 == 1) {
                        System.err.println("[runner] the OpMode ran a whole tick without a hardware call; "
                                + "a loop that never reads or writes hardware makes the simulation non-deterministic");
                    }
                    manager.poll();

                    Protocol.Message cmd = new Protocol.Message(Protocol.COMMAND);
                    cmd.out.writeInt(seq);
                    cmd.out.writeByte(manager.state().code);
                    Protocol.writeString(cmd.out, manager.error());
                    hardware.writeOutputs(cmd.out);
                    SimTelemetry telemetry = manager.telemetry();
                    List<String> lines = telemetry == null ? null : telemetry.takePublished();
                    if (lines == null) {
                        cmd.out.writeByte(0);
                    } else {
                        cmd.out.writeByte(1);
                        cmd.out.writeShort(lines.size());
                        for (String line : lines) Protocol.writeString(cmd.out, line);
                    }
                    cmd.send(out);
                }
            }
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            throw new IOException("interrupted", e);
        } finally {
            manager.stop();
        }
    }
}
