package androidtranscoder.utils;

import android.media.MediaExtractor;
import android.media.MediaFormat;

/* loaded from: classes.dex */
public class MediaExtractorUtils {

    public static class TrackResult {
        public MediaFormat mAudioTrackFormat;
        public int mAudioTrackIndex;
        public String mAudioTrackMime;
        public MediaFormat mVideoTrackFormat;
        public int mVideoTrackIndex;
        public String mVideoTrackMime;

        private TrackResult() {
        }
    }

    private MediaExtractorUtils() {
    }

    public static TrackResult getFirstVideoAndAudioTrack(MediaExtractor mediaExtractor) {
        TrackResult trackResult = new TrackResult();
        trackResult.mVideoTrackIndex = -1;
        trackResult.mAudioTrackIndex = -1;
        int trackCount = mediaExtractor.getTrackCount();
        for (int i = 0; i < trackCount; i++) {
            MediaFormat trackFormat = mediaExtractor.getTrackFormat(i);
            String string = trackFormat.getString("mime");
            if (trackResult.mVideoTrackIndex < 0 && string.startsWith("video/")) {
                trackResult.mVideoTrackIndex = i;
                trackResult.mVideoTrackMime = string;
                trackResult.mVideoTrackFormat = trackFormat;
            } else if (trackResult.mAudioTrackIndex < 0 && string.startsWith("audio/")) {
                trackResult.mAudioTrackIndex = i;
                trackResult.mAudioTrackMime = string;
                trackResult.mAudioTrackFormat = trackFormat;
            }
            if (trackResult.mVideoTrackIndex >= 0 && trackResult.mAudioTrackIndex >= 0) {
                break;
            }
        }
        if (trackResult.mVideoTrackIndex < 0 || trackResult.mAudioTrackIndex < 0) {
            throw new IllegalArgumentException("extractor does not contain video and/or audio tracks.");
        }
        return trackResult;
    }
}