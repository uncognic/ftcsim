package android.util;

/** Android's immutable width and height pair, used for camera resolutions. */
public final class Size {
    private final int mWidth;
    private final int mHeight;

    public Size(int width, int height) {
        mWidth = width;
        mHeight = height;
    }

    public int getWidth() {
        return mWidth;
    }

    public int getHeight() {
        return mHeight;
    }

    /** Parses "640x480" (or "640*480"). */
    public static Size parseSize(String string) throws NumberFormatException {
        int sep = string.indexOf('*');
        if (sep < 0) sep = string.indexOf('x');
        if (sep < 0) throw new NumberFormatException("Invalid Size: \"" + string + "\"");
        return new Size(Integer.parseInt(string.substring(0, sep)), Integer.parseInt(string.substring(sep + 1)));
    }

    @Override
    public boolean equals(Object obj) {
        return obj instanceof Size && ((Size) obj).mWidth == mWidth && ((Size) obj).mHeight == mHeight;
    }

    @Override
    public int hashCode() {
        return mHeight ^ ((mWidth << (Integer.SIZE / 2)) | (mWidth >>> (Integer.SIZE / 2)));
    }

    @Override
    public String toString() {
        return mWidth + "x" + mHeight;
    }
}
