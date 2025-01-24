package androidtranscoder.engine;

import android.media.MediaCodec;
import android.media.MediaExtractor;
import android.media.MediaFormat;
import androidtranscoder.engine.QueuedMuxer;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

/* loaded from: classes.dex */
public class PassThroughTrackTranscoder implements TrackTranscoder {
    static final /* synthetic */ boolean $assertionsDisabled = false;
    private MediaFormat mActualOutputFormat;
    private ByteBuffer mBuffer;
    private final MediaCodec.BufferInfo mBufferInfo = new MediaCodec.BufferInfo();
    private int mBufferSize;
    private final MediaExtractor mExtractor;
    private boolean mIsEOS;
    private final QueuedMuxer mMuxer;
    private final QueuedMuxer.SampleType mSampleType;
    private final int mTrackIndex;
    private long mWrittenPresentationTimeUs;

    public PassThroughTrackTranscoder(MediaExtractor mediaExtractor, int i, QueuedMuxer queuedMuxer, QueuedMuxer.SampleType sampleType) {
        this.mExtractor = mediaExtractor;
        this.mTrackIndex = i;
        this.mMuxer = queuedMuxer;
        this.mSampleType = sampleType;
        MediaFormat trackFormat = mediaExtractor.getTrackFormat(i);
        this.mActualOutputFormat = trackFormat;
        queuedMuxer.setOutputFormat(sampleType, trackFormat);
        int integer = this.mActualOutputFormat.getInteger("max-input-size");
        this.mBufferSize = integer;
        this.mBuffer = ByteBuffer.allocateDirect(integer).order(ByteOrder.nativeOrder());
    }

    @Override // androidtranscoder.engine.TrackTranscoder
    public MediaFormat getDeterminedFormat() {
        return this.mActualOutputFormat;
    }

    @Override // androidtranscoder.engine.TrackTranscoder
    public long getWrittenPresentationTimeUs() {
        return this.mWrittenPresentationTimeUs;
    }

    @Override // androidtranscoder.engine.TrackTranscoder
    public boolean isFinished() {
        return this.mIsEOS;
    }

    @Override // androidtranscoder.engine.TrackTranscoder
    public void release() {
    }

    @Override // androidtranscoder.engine.TrackTranscoder
    public void setup() {
    }

    @Override // androidtranscoder.engine.TrackTranscoder
    public boolean stepPipeline() {
        if (this.mIsEOS) {
            return false;
        }
        int sampleTrackIndex = this.mExtractor.getSampleTrackIndex();
        if (sampleTrackIndex < 0) {
            this.mBuffer.clear();
            this.mBufferInfo.set(0, 0, 0L, 4);
            this.mMuxer.writeSampleData(this.mSampleType, this.mBuffer, this.mBufferInfo);
            this.mIsEOS = true;
            return true;
        }
        if (sampleTrackIndex != this.mTrackIndex) {
            return false;
        }
        this.mBuffer.clear();
        this.mBufferInfo.set(0, this.mExtractor.readSampleData(this.mBuffer, 0), this.mExtractor.getSampleTime(), (this.mExtractor.getSampleFlags() & 1) != 0 ? 1 : 0);
        this.mMuxer.writeSampleData(this.mSampleType, this.mBuffer, this.mBufferInfo);
        this.mWrittenPresentationTimeUs = this.mBufferInfo.presentationTimeUs;
        this.mExtractor.advance();
        return true;
    }
}