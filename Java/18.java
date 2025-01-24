package androidtranscoder.utils;

import java.util.regex.Matcher;
import java.util.regex.Pattern;

/* loaded from: classes.dex */
public class ISO6709LocationParser {
    private final Pattern pattern = Pattern.compile("([+\\-][0-9.]+)([+\\-][0-9.]+)");

    public float[] parse(String str) {
        if (str == null) {
            return null;
        }
        Matcher matcher = this.pattern.matcher(str);
        if (matcher.find() && matcher.groupCount() == 2) {
            try {
                return new float[]{Float.parseFloat(matcher.group(1)), Float.parseFloat(matcher.group(2))};
            } catch (NumberFormatException unused) {
            }
        }
        return null;
    }
}