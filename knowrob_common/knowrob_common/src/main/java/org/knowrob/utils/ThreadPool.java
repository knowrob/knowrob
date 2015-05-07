/*
 * Copyright (c) 2012 Stefan Profanter
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Technische Universiteit Eindhoven nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
*/

package org.knowrob.utils;

import java.util.List;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

/**
 * Simple Thread execution pool to execute multiple callable objects at same time.
 * 
 * @author Stefan Profanter
 * 
 */
public class ThreadPool {

	/**
	 * Executes the given callable objects in a thread pool and returns when all threads have
	 * finished and all callable objects have been executed.
	 * 
	 * @param threads
	 *            list of callable objects
	 */

	public static void executeInPool(List<Callable<Void>> threads) {
		executeInPool(threads, -1);
	}
	public static void executeInPool(List<Callable<Void>> threads, int numParallel) {
		ExecutorService	pool;
		int threadNum = numParallel <= 0 ? Runtime.getRuntime().availableProcessors() * 2 : numParallel;
		pool = Executors.newFixedThreadPool(threadNum);

		try {
			List<Future<Void>> futures = pool.invokeAll(threads);
			for (Future<Void> f : futures) {
				try {
					f.get(); // If called thread threw an exception, get will throw an
								// ExecutionException
				} catch (ExecutionException ex) {
					ex.getCause().printStackTrace(); // Print exceptions from called threads if
														// there were any
				}
			}
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		threads.clear();
	}
}
