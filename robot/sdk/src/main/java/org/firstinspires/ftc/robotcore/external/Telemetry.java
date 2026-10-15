package org.firstinspires.ftc.robotcore.external;

/** Text shown on the Driver Station. Call {@link #update()} to send what you have added. */
public interface Telemetry {
    Item addData(String caption, String format, Object... args);

    Item addData(String caption, Object value);

    <T> Item addData(String caption, Func<T> valueProducer);

    <T> Item addData(String caption, String format, Func<T> valueProducer);

    boolean removeItem(Item item);

    void clear();

    void clearAll();

    boolean update();

    Line addLine();

    Line addLine(String lineCaption);

    boolean removeLine(Line line);

    boolean isAutoClear();

    void setAutoClear(boolean autoClear);

    int getMsTransmissionInterval();

    void setMsTransmissionInterval(int msTransmissionInterval);

    String getItemSeparator();

    void setItemSeparator(String itemSeparator);

    String getCaptionValueSeparator();

    void setCaptionValueSeparator(String captionValueSeparator);

    void speak(String text);

    interface Item {
        String getCaption();

        Item setCaption(String caption);

        Item setValue(String format, Object... args);

        Item setValue(Object value);

        Item setRetained(Boolean retained);

        boolean isRetained();

        Item addData(String caption, String format, Object... args);

        Item addData(String caption, Object value);
    }

    interface Line {
        Item addData(String caption, String format, Object... args);

        Item addData(String caption, Object value);
    }
}
