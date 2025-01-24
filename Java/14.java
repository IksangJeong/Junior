package androidtranscoder.format;

import android.media.MediaFormat;
import io.dcloud.common.util.StringUtil;

/* loaded from: classes.dex */
public class MediaFormatPresets {
    private static final int LONGER_LENGTH_960x540 = 960;

    private MediaFormatPresets() {
    }

    @Deprecated
    public static MediaFormat getExportPreset960x540() {
        MediaFormat createVideoFormat = MediaFormat.createVideoFormat(MediaFormatExtraConstants.MIMETYPE_VIDEO_AVC, LONGER_LENGTH_960x540, 540);
        createVideoFormat.setInteger("bitrate", 5500000);
        createVideoFormat.setInteger("color-format", 2130708361);
        createVideoFormat.setInteger("frame-rate", 30);
        createVideoFormat.setInteger("i-frame-interval", 1);
        return createVideoFormat;
    }

    public static MediaFormat getExportPreset960x540(int i, int i2) {
        int max = Math.max(i, i2);
        int min = Math.min(i, i2);
        int i3 = LONGER_LENGTH_960x540;
        if (max <= LONGER_LENGTH_960x540) {
            return null;
        }
        int i4 = min * LONGER_LENGTH_960x540;
        if (i4 % max == 0) {
            int i5 = i4 / max;
            if (i < i2) {
                i3 = i5;
                i5 = LONGER_LENGTH_960x540;
            }
            MediaFormat createVideoFormat = MediaFormat.createVideoFormat(MediaFormatExtraConstants.MIMETYPE_VIDEO_AVC, i3, i5);
            createVideoFormat.setInteger("bitrate", 5500000);
            createVideoFormat.setInteger("color-format", 2130708361);
            createVideoFormat.setInteger("frame-rate", 30);
            createVideoFormat.setInteger("i-frame-interval", 1);
            return createVideoFormat;
        }
        throw new OutputFormatUnavailableException(StringUtil.format("Could not fit to integer, original: (%d, %d), scaled: (%d, %f)", Integer.valueOf(max), Integer.valueOf(min), Integer.valueOf(LONGER_LENGTH_960x540), Double.valueOf((min * 960.0d) / max)));
    }
}