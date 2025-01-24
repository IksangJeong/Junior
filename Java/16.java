package androidtranscoder.format;

/* loaded from: classes.dex */
public class MediaFormatStrategyPresets {
    public static final int AUDIO_BITRATE_AS_IS = -1;
    public static final int AUDIO_CHANNELS_AS_IS = -1;

    @Deprecated
    public static final MediaFormatStrategy EXPORT_PRESET_960x540 = new ExportPreset960x540Strategy();

    private MediaFormatStrategyPresets() {
    }

    public static MediaFormatStrategy createAndroid720pStrategy(int i, double d) {
        return new Android720pFormatStrategy(i, d);
    }

    public static MediaFormatStrategy createExportPreset960x540Strategy() {
        return new ExportPreset960x540Strategy();
    }
}