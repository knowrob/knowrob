/* 
 * Copyright (c) 2010, Moritz Tenorth
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
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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
 */

package org.knowrob.json_prolog.test;

import org.knowrob.json_prolog.PrologBindings;
import org.knowrob.json_prolog.client.PrologClient;
import org.knowrob.json_prolog.client.PrologQueryProxy;
import org.knowrob.utils.ros.RosUtilities;


public class JSONPrologTestClient {

	public static void main(String args[]) {

	 PrologClient pl = new PrologClient();
	 RosUtilities.runRosjavaNode(pl, new String[]{"org.knowrob.json_prolog.Prolog"});
	 
	 PrologQueryProxy bdgs = pl.query("member(A, [1, 2, 3, 4]), B = ['x', A], C = foo(bar, A, B)");

	 
	 for(PrologBindings bdg : bdgs) {

      System.out.println("Found solution: ");
      System.out.println("A = " + bdg.getBdgs_().get("A") );
      System.out.println("B = " + bdg.getBdgs_().get("B") );
      System.out.println("C = " + bdg.getBdgs_().get("C") );
    }
  }
}