package edu.tum.cs.util;

import java.util.concurrent.TimeUnit;

public class PrintUtil {
	public final static String prettyMillis(long millis) {
		long h, m, s;
		h = TimeUnit.MILLISECONDS.toHours(millis);
		millis -= TimeUnit.HOURS.toMillis(h);

		m = TimeUnit.MILLISECONDS.toMinutes(millis);
		millis -= TimeUnit.MINUTES.toMillis(m);

		s = TimeUnit.MILLISECONDS.toSeconds(millis);
		millis -= TimeUnit.SECONDS.toMillis(s);
		return String.format("%02d:%02d:%02d.%03d", h, m, s, millis);
	}
}
