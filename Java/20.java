package androidtranscoder;

import android.media.MediaFormat;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;
import androidtranscoder.engine.MediaTranscoderEngine;
import androidtranscoder.format.MediaFormatPresets;
import androidtranscoder.format.MediaFormatStrategy;
import java.io.FileDescriptor;
import java.io.FileInputStream;
import java.io.IOException;
import java.util.concurrent.Callable;
import java.util.concurrent.Future;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

/* loaded from: classes.dex */
public class MediaTranscoder {
    private static final int MAXIMUM_THREAD = 1;
    private static final String TAG = "MediaTranscoder";
    private static volatile MediaTranscoder sMediaTranscoder;
    private ThreadPoolExecutor mExecutor = new ThreadPoolExecutor(0, 1, 60, TimeUnit.SECONDS, new LinkedBlockingQueue(), new ThreadFactory() { // from class: androidtranscoder.MediaTranscoder.1
        @Override // java.util.concurrent.ThreadFactory
        public Thread newThread(Runnable runnable) {
            return new Thread(runnable, "MediaTranscoder-Worker");
        }
    });

    /* renamed from: androidtranscoder.MediaTranscoder$4, reason: invalid class name */
    class AnonymousClass4 implements Callable<Void> {
        final /* synthetic */ AtomicReference val$futureReference;
        final /* synthetic */ Handler val$handler;
        final /* synthetic */ FileDescriptor val$inFileDescriptor;
        final /* synthetic */ Listener val$listener;
        final /* synthetic */ MediaFormatStrategy val$outFormatStrategy;
        final /* synthetic */ String val$outPath;

        AnonymousClass4(Handler handler, Listener listener, FileDescriptor fileDescriptor, String str, MediaFormatStrategy mediaFormatStrategy, AtomicReference atomicReference) {
            this.val$handler = handler;
            this.val$listener = listener;
            this.val$inFileDescriptor = fileDescriptor;
            this.val$outPath = str;
            this.val$outFormatStrategy = mediaFormatStrategy;
            this.val$futureReference = atomicReference;
        }

        @Override // java.util.concurrent.Callable
        public Void call() throws Exception {
            try {
                MediaTranscoderEngine mediaTranscoderEngine = new MediaTranscoderEngine();
                mediaTranscoderEngine.setProgressCallback(new MediaTranscoderEngine.ProgressCallback() { // from class: androidtranscoder.MediaTranscoder.4.1
                    @Override // androidtranscoder.engine.MediaTranscoderEngine.ProgressCallback
                    public void onProgress(final double d) {
                        AnonymousClass4.this.val$handler.post(new Runnable() { // from class: androidtranscoder.MediaTranscoder.4.1.1
                            @Override // java.lang.Runnable
                            public void run() {
                                AnonymousClass4.this.val$listener.onTranscodeProgress(d);
                            }
                        });
                    }
                });
                mediaTranscoderEngine.setDataSource(this.val$inFileDescriptor);
                mediaTranscoderEngine.transcodeVideo(this.val$outPath, this.val$outFormatStrategy);
                e = null;
            } catch (IOException e) {
                e = e;
                Log.w(MediaTranscoder.TAG, "Transcode failed: input file (fd: " + this.val$inFileDescriptor.toString() + ") not found or could not open output file ('" + this.val$outPath + "') .", e);
            } catch (InterruptedException e2) {
                e = e2;
                Log.i(MediaTranscoder.TAG, "Cancel transcode video file.", e);
            } catch (RuntimeException e3) {
                e = e3;
                Log.e(MediaTranscoder.TAG, "Fatal error while transcoding, this might be invalid format or bug in engine or Android.", e);
            }
            this.val$handler.post(new Runnable() { // from class: androidtranscoder.MediaTranscoder.4.2
                @Override // java.lang.Runnable
                public void run() {
                    if (e == null) {
                        AnonymousClass4.this.val$listener.onTranscodeCompleted();
                        return;
                    }
                    Future future = (Future) AnonymousClass4.this.val$futureReference.get();
                    if (future == null || !future.isCancelled()) {
                        AnonymousClass4.this.val$listener.onTranscodeFailed(e);
                    } else {
                        AnonymousClass4.this.val$listener.onTranscodeCanceled();
                    }
                }
            });
            if (e == null) {
                return null;
            }
            throw e;
        }
    }

    public interface Listener {
        void onTranscodeCanceled();

        void onTranscodeCompleted();

        void onTranscodeFailed(Exception exc);

        void onTranscodeProgress(double d);
    }

    private MediaTranscoder() {
    }

    public static MediaTranscoder getInstance() {
        if (sMediaTranscoder == null) {
            synchronized (MediaTranscoder.class) {
                if (sMediaTranscoder == null) {
                    sMediaTranscoder = new MediaTranscoder();
                }
            }
        }
        return sMediaTranscoder;
    }

    @Deprecated
    public Future<Void> transcodeVideo(FileDescriptor fileDescriptor, String str, Listener listener) {
        return transcodeVideo(fileDescriptor, str, new MediaFormatStrategy() { // from class: androidtranscoder.MediaTranscoder.2
            @Override // androidtranscoder.format.MediaFormatStrategy
            public MediaFormat createAudioOutputFormat(MediaFormat mediaFormat) {
                return null;
            }

            @Override // androidtranscoder.format.MediaFormatStrategy
            public MediaFormat createVideoOutputFormat(MediaFormat mediaFormat) {
                return MediaFormatPresets.getExportPreset960x540();
            }
        }, listener);
    }

    public Future<Void> transcodeVideo(String str, String str2, MediaFormatStrategy mediaFormatStrategy, final Listener listener) throws IOException {
        final FileInputStream fileInputStream;
        try {
            fileInputStream = new FileInputStream(str);
            try {
                return transcodeVideo(fileInputStream.getFD(), str2, mediaFormatStrategy, new Listener() { // from class: androidtranscoder.MediaTranscoder.3
                    private void closeStream() {
                        try {
                            fileInputStream.close();
                        } catch (IOException e) {
                            Log.e(MediaTranscoder.TAG, "Can't close input stream: ", e);
                        }
                    }

                    @Override // androidtranscoder.MediaTranscoder.Listener
                    public void onTranscodeCanceled() {
                        closeStream();
                        listener.onTranscodeCanceled();
                    }

                    @Override // androidtranscoder.MediaTranscoder.Listener
                    public void onTranscodeCompleted() {
                        closeStream();
                        listener.onTranscodeCompleted();
                    }

                    @Override // androidtranscoder.MediaTranscoder.Listener
                    public void onTranscodeFailed(Exception exc) {
                        closeStream();
                        listener.onTranscodeFailed(exc);
                    }

                    @Override // androidtranscoder.MediaTranscoder.Listener
                    public void onTranscodeProgress(double d) {
                        listener.onTranscodeProgress(d);
                    }
                });
            } catch (IOException e) {
                e = e;
                if (fileInputStream != null) {
                    try {
                        fileInputStream.close();
                    } catch (IOException e2) {
                        Log.e(TAG, "Can't close input stream: ", e2);
                    }
                }
                throw e;
            }
        } catch (IOException e3) {
            e = e3;
            fileInputStream = null;
        }
    }

    public Future<Void> transcodeVideo(FileDescriptor fileDescriptor, String str, MediaFormatStrategy mediaFormatStrategy, Listener listener) {
        Looper myLooper = Looper.myLooper();
        if (myLooper == null) {
            myLooper = Looper.getMainLooper();
        }
        Handler handler = new Handler(myLooper);
        AtomicReference atomicReference = new AtomicReference();
        Future<Void> submit = this.mExecutor.submit(new AnonymousClass4(handler, listener, fileDescriptor, str, mediaFormatStrategy, atomicReference));
        atomicReference.set(submit);
        return submit;
    }
}