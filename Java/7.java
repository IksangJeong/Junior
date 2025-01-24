package androidtranscoder.engine;

import android.media.MediaCodec;
import android.media.MediaFormat;
import android.media.MediaMuxer;
import android.util.Log;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.List;

/* loaded from: classes.dex */
public class QueuedMuxer {
    private static final int BUFFER_SIZE = 65536;
    private static final String TAG = "QueuedMuxer";
    private MediaFormat mAudioFormat;
    private int mAudioTrackIndex;
    private ByteBuffer mByteBuffer;
    private final Listener mListener;
    private final MediaMuxer mMuxer;
    private final List<SampleInfo> mSampleInfoList = new ArrayList();
    private boolean mStarted;
    private MediaFormat mVideoFormat;
    private int mVideoTrackIndex;

    /* renamed from: androidtranscoder.engine.QueuedMuxer$1, reason: invalid class name */
    static /* synthetic */ class AnonymousClass1 {
        static final /* synthetic */ int[] $SwitchMap$androidtranscoder$engine$QueuedMuxer$SampleType;

        static {
            int[] iArr = new int[SampleType.values().length];
            $SwitchMap$androidtranscoder$engine$QueuedMuxer$SampleType = iArr;
            try {
                iArr[SampleType.VIDEO.ordinal()] = 1;
            } catch (NoSuchFieldError unused) {
            }
            try {
                $SwitchMap$androidtranscoder$engine$QueuedMuxer$SampleType[SampleType.AUDIO.ordinal()] = 2;
            } catch (NoSuchFieldError unused2) {
            }
        }
    }

    public interface Listener {
        void onDetermineOutputFormat();
    }

    private static class SampleInfo {
        private final int mFlags;
        private final long mPresentationTimeUs;
        private final SampleType mSampleType;
        private final int mSize;

        /* synthetic */ SampleInfo(SampleType sampleType, int i, MediaCodec.BufferInfo bufferInfo, AnonymousClass1 anonymousClass1) {
            this(sampleType, i, bufferInfo);
        }

        /* JADX INFO: Access modifiers changed from: private */
        public void writeToBufferInfo(MediaCodec.BufferInfo bufferInfo, int i) {
            bufferInfo.set(i, this.mSize, this.mPresentationTimeUs, this.mFlags);
        }

        private SampleInfo(SampleType sampleType, int i, MediaCodec.BufferInfo bufferInfo) {
            this.mSampleType = sampleType;
            this.mSize = i;
            this.mPresentationTimeUs = bufferInfo.presentationTimeUs;
            this.mFlags = bufferInfo.flags;
        }
    }

    public enum SampleType {
        VIDEO,
        AUDIO
    }

    public QueuedMuxer(MediaMuxer mediaMuxer, Listener listener) {
        this.mMuxer = mediaMuxer;
        this.mListener = listener;
    }

    private int getTrackIndexForSampleType(SampleType sampleType) {
        int i = AnonymousClass1.$SwitchMap$androidtranscoder$engine$QueuedMuxer$SampleType[sampleType.ordinal()];
        if (i == 1) {
            return this.mVideoTrackIndex;
        }
        if (i == 2) {
            return this.mAudioTrackIndex;
        }
        throw new AssertionError();
    }

    private void onSetOutputFormat() {
        if (this.mVideoFormat == null || this.mAudioFormat == null) {
            return;
        }
        this.mListener.onDetermineOutputFormat();
        this.mVideoTrackIndex = this.mMuxer.addTrack(this.mVideoFormat);
        Log.v(TAG, "Added track #" + this.mVideoTrackIndex + " with " + this.mVideoFormat.getString("mime") + " to muxer");
        this.mAudioTrackIndex = this.mMuxer.addTrack(this.mAudioFormat);
        Log.v(TAG, "Added track #" + this.mAudioTrackIndex + " with " + this.mAudioFormat.getString("mime") + " to muxer");
        this.mMuxer.start();
        this.mStarted = true;
        int i = 0;
        if (this.mByteBuffer == null) {
            this.mByteBuffer = ByteBuffer.allocate(0);
        }
        this.mByteBuffer.flip();
        Log.v(TAG, "Output format determined, writing " + this.mSampleInfoList.size() + " samples / " + this.mByteBuffer.limit() + " bytes to muxer.");
        MediaCodec.BufferInfo bufferInfo = new MediaCodec.BufferInfo();
        for (SampleInfo sampleInfo : this.mSampleInfoList) {
            sampleInfo.writeToBufferInfo(bufferInfo, i);
            this.mMuxer.writeSampleData(getTrackIndexForSampleType(sampleInfo.mSampleType), this.mByteBuffer, bufferInfo);
            i += sampleInfo.mSize;
        }
        this.mSampleInfoList.clear();
        this.mByteBuffer = null;
    }

    public void setOutputFormat(SampleType sampleType, MediaFormat mediaFormat) {
        int i = AnonymousClass1.$SwitchMap$androidtranscoder$engine$QueuedMuxer$SampleType[sampleType.ordinal()];
        if (i == 1) {
            this.mVideoFormat = mediaFormat;
        } else {
            if (i != 2) {
                throw new AssertionError();
            }
            this.mAudioFormat = mediaFormat;
        }
        onSetOutputFormat();
    }

    public void writeSampleData(SampleType sampleType, ByteBuffer byteBuffer, MediaCodec.BufferInfo bufferInfo) {
        if (this.mStarted) {
            this.mMuxer.writeSampleData(getTrackIndexForSampleType(sampleType), byteBuffer, bufferInfo);
            return;
        }
        byteBuffer.limit(bufferInfo.offset + bufferInfo.size);
        byteBuffer.position(bufferInfo.offset);
        if (this.mByteBuffer == null) {
            this.mByteBuffer = ByteBuffer.allocateDirect(65536).order(ByteOrder.nativeOrder());
        }
        this.mByteBuffer.put(byteBuffer);
        this.mSampleInfoList.add(new SampleInfo(sampleType, bufferInfo.size, bufferInfo, null));
    }
}