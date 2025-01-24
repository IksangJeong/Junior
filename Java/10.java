package androidtranscoder.engine;

import android.media.MediaCodec;
import android.media.MediaCrypto;
import android.media.MediaExtractor;
import android.media.MediaFormat;
import android.view.Surface;
import androidtranscoder.engine.QueuedMuxer;
import androidtranscoder.format.MediaFormatExtraConstants;
import java.io.IOException;
import java.nio.ByteBuffer;

/* loaded from: classes.dex */
public class VideoTrackTranscoder implements TrackTranscoder {
    private static final int DRAIN_STATE_CONSUMED = 2;
    private static final int DRAIN_STATE_NONE = 0;
    private static final int DRAIN_STATE_SHOULD_RETRY_IMMEDIATELY = 1;
    private static final String TAG = "VideoTrackTranscoder";
    private int frameRate;
    private MediaFormat mActualOutputFormat;
    private MediaCodec mDecoder;
    private ByteBuffer[] mDecoderInputBuffers;
    private OutputSurface mDecoderOutputSurfaceWrapper;
    private boolean mDecoderStarted;
    private MediaCodec mEncoder;
    private InputSurface mEncoderInputSurfaceWrapper;
    private ByteBuffer[] mEncoderOutputBuffers;
    private boolean mEncoderStarted;
    private final MediaExtractor mExtractor;
    private boolean mIsDecoderEOS;
    private boolean mIsEncoderEOS;
    private boolean mIsExtractorEOS;
    private final QueuedMuxer mMuxer;
    private final MediaFormat mOutputFormat;
    private final int mTrackIndex;
    private long mWrittenPresentationTimeUs;
    private final MediaCodec.BufferInfo mBufferInfo = new MediaCodec.BufferInfo();
    int currentSecond = 0;
    int currentFrameCount = 0;

    public VideoTrackTranscoder(MediaExtractor mediaExtractor, int i, MediaFormat mediaFormat, QueuedMuxer queuedMuxer) {
        this.mExtractor = mediaExtractor;
        this.mTrackIndex = i;
        this.mOutputFormat = mediaFormat;
        this.mMuxer = queuedMuxer;
    }

    private int drainDecoder(long j) {
        if (this.mIsDecoderEOS) {
            return 0;
        }
        int dequeueOutputBuffer = this.mDecoder.dequeueOutputBuffer(this.mBufferInfo, j);
        if (dequeueOutputBuffer == -3 || dequeueOutputBuffer == -2) {
            return 1;
        }
        if (dequeueOutputBuffer == -1) {
            return 0;
        }
        if ((this.mBufferInfo.flags & 4) != 0) {
            this.mEncoder.signalEndOfInputStream();
            this.mIsDecoderEOS = true;
            this.mBufferInfo.size = 0;
        }
        boolean z = this.mBufferInfo.size > 0;
        this.mDecoder.releaseOutputBuffer(dequeueOutputBuffer, z);
        if (!z) {
            return 2;
        }
        this.mDecoderOutputSurfaceWrapper.awaitNewImage();
        int i = (int) (this.mBufferInfo.presentationTimeUs / 1000000);
        if (this.currentSecond == i) {
            this.currentFrameCount++;
        } else {
            this.currentSecond = i;
            this.currentFrameCount = 0;
        }
        int i2 = this.frameRate;
        if (i2 <= 30) {
            this.mDecoderOutputSurfaceWrapper.drawImage();
            this.mEncoderInputSurfaceWrapper.setPresentationTime(this.mBufferInfo.presentationTimeUs * 1000);
            this.mEncoderInputSurfaceWrapper.swapBuffers();
            return 2;
        }
        if (i2 <= 50) {
            if (this.currentFrameCount % 5 == 0) {
                return 2;
            }
            this.mDecoderOutputSurfaceWrapper.drawImage();
            this.mEncoderInputSurfaceWrapper.setPresentationTime(this.mBufferInfo.presentationTimeUs * 1000);
            this.mEncoderInputSurfaceWrapper.swapBuffers();
            return 2;
        }
        if (this.currentFrameCount % 3 == 0) {
            return 2;
        }
        this.mDecoderOutputSurfaceWrapper.drawImage();
        this.mEncoderInputSurfaceWrapper.setPresentationTime(this.mBufferInfo.presentationTimeUs * 1000);
        this.mEncoderInputSurfaceWrapper.swapBuffers();
        return 2;
    }

    private int drainEncoder(long j) {
        if (this.mIsEncoderEOS) {
            return 0;
        }
        int dequeueOutputBuffer = this.mEncoder.dequeueOutputBuffer(this.mBufferInfo, j);
        if (dequeueOutputBuffer == -3) {
            this.mEncoderOutputBuffers = this.mEncoder.getOutputBuffers();
            return 1;
        }
        if (dequeueOutputBuffer == -2) {
            if (this.mActualOutputFormat != null) {
                throw new RuntimeException("Video output format changed twice.");
            }
            MediaFormat outputFormat = this.mEncoder.getOutputFormat();
            this.mActualOutputFormat = outputFormat;
            this.mMuxer.setOutputFormat(QueuedMuxer.SampleType.VIDEO, outputFormat);
            return 1;
        }
        if (dequeueOutputBuffer == -1) {
            return 0;
        }
        if (this.mActualOutputFormat == null) {
            throw new RuntimeException("Could not determine actual output format.");
        }
        MediaCodec.BufferInfo bufferInfo = this.mBufferInfo;
        int i = bufferInfo.flags;
        if ((i & 4) != 0) {
            this.mIsEncoderEOS = true;
            bufferInfo.set(0, 0, 0L, i);
        }
        MediaCodec.BufferInfo bufferInfo2 = this.mBufferInfo;
        if ((bufferInfo2.flags & 2) != 0) {
            this.mEncoder.releaseOutputBuffer(dequeueOutputBuffer, false);
            return 1;
        }
        this.mMuxer.writeSampleData(QueuedMuxer.SampleType.VIDEO, this.mEncoderOutputBuffers[dequeueOutputBuffer], bufferInfo2);
        this.mWrittenPresentationTimeUs = this.mBufferInfo.presentationTimeUs;
        this.mEncoder.releaseOutputBuffer(dequeueOutputBuffer, false);
        return 2;
    }

    private int drainExtractor(long j) {
        int dequeueInputBuffer;
        if (this.mIsExtractorEOS) {
            return 0;
        }
        int sampleTrackIndex = this.mExtractor.getSampleTrackIndex();
        if ((sampleTrackIndex >= 0 && sampleTrackIndex != this.mTrackIndex) || (dequeueInputBuffer = this.mDecoder.dequeueInputBuffer(j)) < 0) {
            return 0;
        }
        if (sampleTrackIndex < 0) {
            this.mIsExtractorEOS = true;
            this.mDecoder.queueInputBuffer(dequeueInputBuffer, 0, 0, 0L, 4);
            return 0;
        }
        this.mDecoder.queueInputBuffer(dequeueInputBuffer, 0, this.mExtractor.readSampleData(this.mDecoderInputBuffers[dequeueInputBuffer], 0), this.mExtractor.getSampleTime(), (this.mExtractor.getSampleFlags() & 1) != 0 ? 1 : 0);
        this.mExtractor.advance();
        return 2;
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
        return this.mIsEncoderEOS;
    }

    @Override // androidtranscoder.engine.TrackTranscoder
    public void release() {
        OutputSurface outputSurface = this.mDecoderOutputSurfaceWrapper;
        if (outputSurface != null) {
            outputSurface.release();
            this.mDecoderOutputSurfaceWrapper = null;
        }
        InputSurface inputSurface = this.mEncoderInputSurfaceWrapper;
        if (inputSurface != null) {
            inputSurface.release();
            this.mEncoderInputSurfaceWrapper = null;
        }
        MediaCodec mediaCodec = this.mDecoder;
        if (mediaCodec != null) {
            if (this.mDecoderStarted) {
                mediaCodec.stop();
            }
            this.mDecoder.release();
            this.mDecoder = null;
        }
        MediaCodec mediaCodec2 = this.mEncoder;
        if (mediaCodec2 != null) {
            if (this.mEncoderStarted) {
                mediaCodec2.stop();
            }
            this.mEncoder.release();
            this.mEncoder = null;
        }
    }

    @Override // androidtranscoder.engine.TrackTranscoder
    public void setup() {
        this.mExtractor.selectTrack(this.mTrackIndex);
        try {
            MediaCodec createEncoderByType = MediaCodec.createEncoderByType(this.mOutputFormat.getString("mime"));
            this.mEncoder = createEncoderByType;
            createEncoderByType.configure(this.mOutputFormat, (Surface) null, (MediaCrypto) null, 1);
            InputSurface inputSurface = new InputSurface(this.mEncoder.createInputSurface());
            this.mEncoderInputSurfaceWrapper = inputSurface;
            inputSurface.makeCurrent();
            this.mEncoder.start();
            this.mEncoderStarted = true;
            this.mEncoderOutputBuffers = this.mEncoder.getOutputBuffers();
            MediaFormat trackFormat = this.mExtractor.getTrackFormat(this.mTrackIndex);
            if (trackFormat.containsKey(MediaFormatExtraConstants.KEY_ROTATION_DEGREES)) {
                trackFormat.setInteger(MediaFormatExtraConstants.KEY_ROTATION_DEGREES, 0);
            }
            if (trackFormat.getString("mime").startsWith("video/") && trackFormat.containsKey("frame-rate")) {
                this.frameRate = trackFormat.getInteger("frame-rate");
            }
            this.mDecoderOutputSurfaceWrapper = new OutputSurface();
            try {
                MediaCodec createDecoderByType = MediaCodec.createDecoderByType(trackFormat.getString("mime"));
                this.mDecoder = createDecoderByType;
                createDecoderByType.configure(trackFormat, this.mDecoderOutputSurfaceWrapper.getSurface(), (MediaCrypto) null, 0);
                this.mDecoder.start();
                this.mDecoderStarted = true;
                this.mDecoderInputBuffers = this.mDecoder.getInputBuffers();
            } catch (IOException e) {
                throw new IllegalStateException(e);
            }
        } catch (IOException e2) {
            throw new IllegalStateException(e2);
        }
    }

    @Override // androidtranscoder.engine.TrackTranscoder
    public boolean stepPipeline() {
        int drainDecoder;
        boolean z = false;
        while (drainEncoder(0L) != 0) {
            z = true;
        }
        do {
            drainDecoder = drainDecoder(0L);
            if (drainDecoder != 0) {
                z = true;
            }
        } while (drainDecoder == 1);
        while (drainExtractor(0L) != 0) {
            z = true;
        }
        return z;
    }
}