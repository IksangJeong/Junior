package androidtranscoder.engine;

import android.media.MediaFormat;

/* loaded from: classes.dex */
public interface TrackTranscoder {
    MediaFormat getDeterminedFormat();

    long getWrittenPresentationTimeUs();

    boolean isFinished();

    void release();

    void setup();

    boolean stepPipeline();
}