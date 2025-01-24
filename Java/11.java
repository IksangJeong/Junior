package androidtranscoder.format;

import android.media.MediaFormat;

/* loaded from: classes.dex */
class Android720pFormatStrategy implements MediaFormatStrategy {
    public static final int AUDIO_BITRATE_AS_IS = -1;
    public static final int AUDIO_CHANNELS_AS_IS = -1;
    private static final int DEFAULT_VIDEO_BITRATE = 8000000;
    private static final int LONGER_LENGTH = 1280;
    private static final int SHORTER_LENGTH = 720;
    private static final String TAG = "720pFormatStrategy";
    private int compressLevel;
    private final int mAudioBitrate = 128000;
    private final int mAudioChannels = 1;
    private int mVideoBitrate;
    private double resolution;

    public Android720pFormatStrategy(int i, double d) {
        this.compressLevel = 1;
        this.resolution = 1.0d;
        this.compressLevel = i;
        this.resolution = d;
    }

    @Override // androidtranscoder.format.MediaFormatStrategy
    public MediaFormat createAudioOutputFormat(MediaFormat mediaFormat) {
        if (this.mAudioBitrate == -1 || this.mAudioChannels == -1) {
            return null;
        }
        MediaFormat createAudioFormat = MediaFormat.createAudioFormat(MediaFormatExtraConstants.MIMETYPE_AUDIO_AAC, mediaFormat.getInteger("sample-rate"), this.mAudioChannels);
        createAudioFormat.setInteger("aac-profile", 2);
        createAudioFormat.setInteger("bitrate", this.mAudioBitrate);
        return createAudioFormat;
    }

    @Override // androidtranscoder.format.MediaFormatStrategy
    public MediaFormat createVideoOutputFormat(MediaFormat mediaFormat) {
        int i;
        int i2;
        double d;
        double d2;
        int integer = mediaFormat.getInteger("width");
        int integer2 = mediaFormat.getInteger("height");
        double d3 = this.resolution;
        if (d3 == 1.0d) {
            int i3 = this.compressLevel;
            if (i3 == 1) {
                d = integer;
                d2 = 0.8d;
            } else if (i3 == 2) {
                d = integer;
                d2 = 0.5d;
            } else {
                d = integer;
                d2 = 0.3d;
            }
            i = (int) (d * d2);
            i2 = (int) (integer2 * d2);
        } else {
            i = (int) (integer * d3);
            i2 = (int) (integer2 * d3);
        }
        if (i % 2 > 0) {
            i++;
        }
        if (i2 % 2 > 0) {
            i2++;
        }
        this.mVideoBitrate = i * i2;
        MediaFormat createVideoFormat = MediaFormat.createVideoFormat(MediaFormatExtraConstants.MIMETYPE_VIDEO_AVC, i, i2);
        createVideoFormat.setInteger("bitrate", this.mVideoBitrate);
        createVideoFormat.setInteger("frame-rate", 25);
        createVideoFormat.setInteger("i-frame-interval", 3);
        createVideoFormat.setInteger("color-format", 2130708361);
        return createVideoFormat;
    }
}