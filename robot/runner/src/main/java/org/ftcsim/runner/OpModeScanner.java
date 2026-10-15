package org.ftcsim.runner;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.io.File;
import java.io.IOException;
import java.lang.reflect.Modifier;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.Enumeration;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.jar.JarEntry;
import java.util.jar.JarFile;
import java.util.stream.Stream;

/** Finds {@code @TeleOp} and {@code @Autonomous} classes on the classpath, like the Robot Controller does. */
final class OpModeScanner {
    static final class Entry {
        final String name;
        final String group;
        final boolean autonomous;
        final Class<? extends OpMode> type;

        Entry(String name, String group, boolean autonomous, Class<? extends OpMode> type) {
            this.name = name;
            this.group = group;
            this.autonomous = autonomous;
            this.type = type;
        }
    }

    private static final String[] SKIP = {"com/qualcomm/", "org/firstinspires/ftc/robotcore/", "org/ftcsim/", "META-INF/"};

    private OpModeScanner() {
    }

    static List<Entry> scan() {
        Set<String> classNames = new HashSet<>();
        for (String element : System.getProperty("java.class.path").split(File.pathSeparator)) {
            File f = new File(element);
            try {
                if (f.isDirectory()) scanDirectory(f.toPath(), classNames);
                else if (f.getName().endsWith(".jar")) scanJar(f, classNames);
            } catch (IOException e) {
                System.err.println("[runner] could not scan " + f + ": " + e.getMessage());
            }
        }

        List<Entry> found = new ArrayList<>();
        Set<String> names = new HashSet<>();
        ClassLoader loader = OpModeScanner.class.getClassLoader();
        for (String className : classNames) {
            Class<?> c;
            try {
                c = Class.forName(className, false, loader);
            } catch (Throwable t) {
                continue;
            }
            if (!OpMode.class.isAssignableFrom(c) || Modifier.isAbstract(c.getModifiers()) || c.isAnnotationPresent(Disabled.class)) {
                continue;
            }
            TeleOp teleOp = c.getAnnotation(TeleOp.class);
            Autonomous auto = c.getAnnotation(Autonomous.class);
            if (teleOp == null && auto == null) continue;
            String name = teleOp != null ? teleOp.name() : auto.name();
            String group = teleOp != null ? teleOp.group() : auto.group();
            if (name.isEmpty()) name = c.getSimpleName();
            if (!names.add(name)) {
                System.err.println("[runner] skipping " + c.getName() + ": another OpMode is already named \"" + name + "\"");
                continue;
            }
            @SuppressWarnings("unchecked") Class<? extends OpMode> type = (Class<? extends OpMode>) c;
            found.add(new Entry(name, group, auto != null, type));
        }
        found.sort(Comparator.<Entry, Boolean>comparing(e -> !e.autonomous).thenComparing(e -> e.group).thenComparing(e -> e.name));
        return found;
    }

    private static void scanDirectory(Path root, Set<String> out) throws IOException {
        try (Stream<Path> files = Files.walk(root)) {
            files.filter(p -> p.toString().endsWith(".class"))
                    .forEach(p -> add(root.relativize(p).toString().replace(File.separatorChar, '/'), out));
        }
    }

    private static void scanJar(File jar, Set<String> out) throws IOException {
        try (JarFile jf = new JarFile(jar)) {
            for (Enumeration<JarEntry> e = jf.entries(); e.hasMoreElements(); ) {
                String n = e.nextElement().getName();
                if (n.endsWith(".class")) add(n, out);
            }
        }
    }

    private static void add(String path, Set<String> out) {
        for (String prefix : SKIP) if (path.startsWith(prefix)) return;
        if (path.endsWith("module-info.class") || path.contains("$")) return;
        out.add(path.substring(0, path.length() - ".class".length()).replace('/', '.'));
    }
}
