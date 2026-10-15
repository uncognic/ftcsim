package org.ftcsim.runner;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/** Driver Station telemetry. {@link #update()} publishes the current lines to the simulator. */
final class SimTelemetry implements Telemetry {
    /** More than a Driver Station screen can show; also keeps one update from flooding the link. */
    static final int MAX_LINES = 200;
    static final int MAX_LINE_LENGTH = 500;

    private final List<Object> entries = new ArrayList<>(); // ItemImpl or LineImpl
    private volatile List<String> published;
    private boolean autoClear = true;
    private int msTransmissionInterval = 250;
    private String itemSeparator = " | ";
    private String captionValueSeparator = " : ";

    /** Lines published since the last call, or null. Called by the runner. */
    List<String> takePublished() {
        List<String> p = published;
        published = null;
        return p;
    }

    private final class ItemImpl implements Item {
        String caption;
        Object value;
        Func<?> producer;
        String format;
        boolean retained;

        @Override
        public String getCaption() {
            return caption;
        }

        @Override
        public Item setCaption(String caption) {
            this.caption = caption;
            return this;
        }

        @Override
        public Item setValue(String format, Object... args) {
            this.value = String.format(Locale.US, format, args);
            return this;
        }

        @Override
        public Item setValue(Object value) {
            this.value = value;
            return this;
        }

        @Override
        public Item setRetained(Boolean retained) {
            this.retained = retained != null && retained;
            return this;
        }

        @Override
        public boolean isRetained() {
            return retained || producer != null;
        }

        @Override
        public Item addData(String caption, String format, Object... args) {
            return SimTelemetry.this.addData(caption, format, args);
        }

        @Override
        public Item addData(String caption, Object value) {
            return SimTelemetry.this.addData(caption, value);
        }

        String render() {
            Object v = value;
            if (producer != null) {
                Object produced = producer.value();
                v = format != null ? String.format(Locale.US, format, produced) : produced;
            }
            return caption + captionValueSeparator + v;
        }
    }

    private final class LineImpl implements Line {
        final String caption;
        final List<ItemImpl> items = new ArrayList<>();

        LineImpl(String caption) {
            this.caption = caption;
        }

        @Override
        public Item addData(String caption, String format, Object... args) {
            return add(caption).setValue(format, args);
        }

        @Override
        public Item addData(String caption, Object value) {
            return add(caption).setValue(value);
        }

        private ItemImpl add(String caption) {
            ItemImpl item = new ItemImpl();
            item.caption = caption;
            items.add(item);
            return item;
        }

        String render() {
            StringBuilder sb = new StringBuilder(caption);
            for (int i = 0; i < items.size(); i++) {
                if (i > 0 || !caption.isEmpty()) sb.append(i == 0 ? " " : itemSeparator);
                sb.append(items.get(i).render());
            }
            return sb.toString();
        }
    }

    private synchronized ItemImpl newItem(String caption) {
        ItemImpl item = new ItemImpl();
        item.caption = caption;
        entries.add(item);
        return item;
    }

    @Override
    public Item addData(String caption, String format, Object... args) {
        return newItem(caption).setValue(format, args);
    }

    @Override
    public Item addData(String caption, Object value) {
        return newItem(caption).setValue(value);
    }

    @Override
    public <T> Item addData(String caption, Func<T> valueProducer) {
        ItemImpl item = newItem(caption);
        item.producer = valueProducer;
        return item;
    }

    @Override
    public <T> Item addData(String caption, String format, Func<T> valueProducer) {
        ItemImpl item = newItem(caption);
        item.producer = valueProducer;
        item.format = format;
        return item;
    }

    @Override
    public synchronized boolean removeItem(Item item) {
        return entries.remove(item);
    }

    @Override
    public synchronized void clear() {
        entries.removeIf(e -> !(e instanceof ItemImpl) || !((ItemImpl) e).isRetained());
    }

    @Override
    public synchronized void clearAll() {
        entries.clear();
        published = new ArrayList<>();
    }

    @Override
    public synchronized boolean update() {
        List<String> lines = new ArrayList<>();
        for (Object e : entries) {
            if (lines.size() == MAX_LINES) {
                lines.add("(" + (entries.size() - MAX_LINES) + " more lines not shown: telemetry.update() was called with "
                        + entries.size() + " items)");
                break;
            }
            String line = e instanceof ItemImpl ? ((ItemImpl) e).render() : ((LineImpl) e).render();
            lines.add(line.length() > MAX_LINE_LENGTH ? line.substring(0, MAX_LINE_LENGTH) + "..." : line);
        }
        published = lines;
        if (autoClear) clear();
        return true;
    }

    @Override
    public Line addLine() {
        return addLine("");
    }

    @Override
    public synchronized Line addLine(String lineCaption) {
        LineImpl line = new LineImpl(lineCaption);
        entries.add(line);
        return line;
    }

    @Override
    public synchronized boolean removeLine(Line line) {
        for (Iterator<Object> it = entries.iterator(); it.hasNext(); ) {
            if (it.next() == line) {
                it.remove();
                return true;
            }
        }
        return false;
    }

    @Override
    public boolean isAutoClear() {
        return autoClear;
    }

    @Override
    public void setAutoClear(boolean autoClear) {
        this.autoClear = autoClear;
    }

    @Override
    public int getMsTransmissionInterval() {
        return msTransmissionInterval;
    }

    @Override
    public void setMsTransmissionInterval(int msTransmissionInterval) {
        this.msTransmissionInterval = msTransmissionInterval;
    }

    @Override
    public String getItemSeparator() {
        return itemSeparator;
    }

    @Override
    public void setItemSeparator(String itemSeparator) {
        this.itemSeparator = itemSeparator;
    }

    @Override
    public String getCaptionValueSeparator() {
        return captionValueSeparator;
    }

    @Override
    public void setCaptionValueSeparator(String captionValueSeparator) {
        this.captionValueSeparator = captionValueSeparator;
    }

    @Override
    public void speak(String text) {
    }
}
