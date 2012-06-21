/*******************************************************************************
 * Copyright (c) 2012 Stefan Profanter.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Public License v3.0
 * which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors:
 *     Stefan Profanter - initial API and implementation, Year: 2012
 ******************************************************************************/
package edu.tum.cs.util;

import java.util.concurrent.TimeUnit;

/**
 * Util functions for printing on screen.
 * 
 * 
 * @author Stefan Profanter
 * 
 */
public class PrintUtil {
	/**
	 * Convert millisecond value to human readable string hh:mm:ss.xxx
	 * 
	 * @param mil
	 *            Time in milliseconds
	 * @return String in format hh:mm:dd.sss
	 */
	public final static String prettyMillis(long mil) {
		long h, m, s;
		long millis = mil;
		h = TimeUnit.MILLISECONDS.toHours(millis);
		millis -= TimeUnit.HOURS.toMillis(h);

		m = TimeUnit.MILLISECONDS.toMinutes(millis);
		millis -= TimeUnit.MINUTES.toMillis(m);

		s = TimeUnit.MILLISECONDS.toSeconds(millis);
		millis -= TimeUnit.SECONDS.toMillis(s);
		return String.format("%02d:%02d:%02d.%03d", h, m, s, millis);
	}
}
