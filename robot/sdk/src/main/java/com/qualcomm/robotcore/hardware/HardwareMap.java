package com.qualcomm.robotcore.hardware;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

/** Devices from the active robot configuration, looked up by the names given in that configuration. */
public class HardwareMap implements Iterable<HardwareDevice> {
    public final DeviceMapping<DcMotor> dcMotor = new DeviceMapping<>(DcMotor.class);
    public final DeviceMapping<Servo> servo = new DeviceMapping<>(Servo.class);
    public final DeviceMapping<CRServo> crservo = new DeviceMapping<>(CRServo.class);
    public final DeviceMapping<VoltageSensor> voltageSensor = new DeviceMapping<>(VoltageSensor.class);

    private final Map<String, List<HardwareDevice>> allDevicesMap = new LinkedHashMap<>();
    private final List<HardwareDevice> allDevices = new ArrayList<>();

    public <T> T get(Class<? extends T> classOrInterface, String deviceName) {
        T result = tryGet(classOrInterface, deviceName);
        if (result == null) {
            throw new IllegalArgumentException(String.format(
                    "Unable to find a hardware device with name \"%s\" and type %s", deviceName, classOrInterface.getSimpleName()));
        }
        return result;
    }

    public <T> T tryGet(Class<? extends T> classOrInterface, String deviceName) {
        List<HardwareDevice> list = allDevicesMap.get(deviceName.trim());
        if (list != null) {
            for (HardwareDevice device : list) {
                if (classOrInterface.isInstance(device)) {
                    @SuppressWarnings("unchecked") T t = (T) device;
                    return t;
                }
            }
        }
        return null;
    }

    public HardwareDevice get(String deviceName) {
        List<HardwareDevice> list = allDevicesMap.get(deviceName.trim());
        if (list == null || list.isEmpty()) {
            throw new IllegalArgumentException(String.format("Unable to find a hardware device with the name \"%s\"", deviceName));
        }
        return list.get(0);
    }

    public <T> List<T> getAll(Class<? extends T> classOrInterface) {
        List<T> result = new ArrayList<>();
        for (HardwareDevice device : allDevices) {
            if (classOrInterface.isInstance(device)) {
                @SuppressWarnings("unchecked") T t = (T) device;
                result.add(t);
            }
        }
        return result;
    }

    public void put(String deviceName, HardwareDevice device) {
        deviceName = deviceName.trim();
        allDevicesMap.computeIfAbsent(deviceName, k -> new ArrayList<>()).add(device);
        allDevices.add(device);
        dcMotor.putIfType(deviceName, device);
        servo.putIfType(deviceName, device);
        crservo.putIfType(deviceName, device);
        voltageSensor.putIfType(deviceName, device);
    }

    public Set<String> getNamesOf(HardwareDevice device) {
        Set<String> names = new LinkedHashSet<>();
        for (Map.Entry<String, List<HardwareDevice>> e : allDevicesMap.entrySet()) {
            if (e.getValue().contains(device)) names.add(e.getKey());
        }
        return names;
    }

    public int size() {
        return allDevices.size();
    }

    @Override
    public Iterator<HardwareDevice> iterator() {
        return Collections.unmodifiableList(allDevices).iterator();
    }

    public class DeviceMapping<DEVICE_TYPE extends HardwareDevice> implements Iterable<DEVICE_TYPE> {
        private final Map<String, DEVICE_TYPE> map = new LinkedHashMap<>();
        private final Class<DEVICE_TYPE> deviceTypeClass;

        public DeviceMapping(Class<DEVICE_TYPE> deviceTypeClass) {
            this.deviceTypeClass = deviceTypeClass;
        }

        public Class<DEVICE_TYPE> getDeviceTypeClass() {
            return deviceTypeClass;
        }

        public DEVICE_TYPE get(String deviceName) {
            DEVICE_TYPE device = map.get(deviceName.trim());
            if (device == null) {
                throw new IllegalArgumentException(String.format("Unable to find a hardware device with the name \"%s\"", deviceName));
            }
            return device;
        }

        public void put(String deviceName, DEVICE_TYPE device) {
            map.put(deviceName.trim(), device);
        }

        void putIfType(String deviceName, HardwareDevice device) {
            if (deviceTypeClass.isInstance(device)) map.put(deviceName, deviceTypeClass.cast(device));
        }

        public boolean contains(String deviceName) {
            return map.containsKey(deviceName.trim());
        }

        public Set<Map.Entry<String, DEVICE_TYPE>> entrySet() {
            return Collections.unmodifiableMap(map).entrySet();
        }

        public int size() {
            return map.size();
        }

        @Override
        public Iterator<DEVICE_TYPE> iterator() {
            return Collections.unmodifiableCollection(map.values()).iterator();
        }
    }
}
