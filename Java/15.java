package androidtranscoder.format;

import android.media.MediaFormat;

/* loaded from: classes.dex */
public interface MediaFormatStrategy {
    MediaFormat createAudioOutputFormat(MediaFormat mediaFormat);

    MediaFormat createVideoOutputFormat(MediaFormat mediaFormat);
}