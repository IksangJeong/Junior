package androidtranscoder.engine;

import android.media.MediaFormat;
import androidtranscoder.format.MediaFormatExtraConstants;

/* loaded from: classes.dex */
class MediaFormatValidator {
    MediaFormatValidator() {
    }

    public static void validateAudioOutputFormat(MediaFormat mediaFormat) {
        String string = mediaFormat.getString("mime");
        if (MediaFormatExtraConstants.MIMETYPE_AUDIO_AAC.equals(string)) {
            return;
        }
        throw new InvalidOutputFormatException("Audio codecs other than AAC is not supported, actual mime type: " + string);
    }

    public static void validateVideoOutputFormat(MediaFormat mediaFormat) {
        String string = mediaFormat.getString("mime");
        if (MediaFormatExtraConstants.MIMETYPE_VIDEO_AVC.equals(string)) {
            return;
        }
        throw new InvalidOutputFormatException("Video codecs other than AVC is not supported, actual mime type: " + string);
    }
}