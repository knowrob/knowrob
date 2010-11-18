/*
 * Created on Nov 15, 2007
 */
package edu.tum.cs.util.math;


/**
 * The class <code>MathUtils</code> contains some functions from
 * org.apache.commons.math to calculate the error function. 
 */
public class MathUtils {
	
	/**
	 * Provides a generic means to evaluate continued fractions.  Subclasses simply
	 * provided the a and b coefficients to evaluate the continued fraction.
	 *
	 * <p>
	 * References:
	 * <ul>
	 * <li><a href="http://mathworld.wolfram.com/ContinuedFraction.html">
	 * Continued Fraction</a></li>
	 * </ul>
	 * </p>
	 *
	 * @version $Revision: 1.14 $ $Date: 2004/06/23 16:26:16 $
	 */
	public static abstract class ContinuedFraction {
	    
	    /** Maximum allowed numerical error. */
	    private static final double DEFAULT_EPSILON = 10e-9;

	    /**
	     * Default constructor.
	     */
	    protected ContinuedFraction() {
	        super();
	    }

	    /**
	     * Access the n-th a coefficient of the continued fraction.  Since a can be
	     * a function of the evaluation point, x, that is passed in as well.
	     * @param n the coefficient index to retrieve.
	     * @param x the evaluation point.
	     * @return the n-th a coefficient.
	     */
	    protected abstract double getA(int n, double x);

	    /**
	     * Access the n-th b coefficient of the continued fraction.  Since b can be
	     * a function of the evaluation point, x, that is passed in as well.
	     * @param n the coefficient index to retrieve.
	     * @param x the evaluation point.
	     * @return the n-th b coefficient.
	     */
	    protected abstract double getB(int n, double x);

	    /**
	     * Evaluates the continued fraction at the value x.
	     * @param x the evaluation point.
	     * @return the value of the continued fraction evaluated at x. 
	     * @throws MathException if the algorithm fails to converge.
	     */
	    public double evaluate(double x) {
	        return evaluate(x, DEFAULT_EPSILON, Integer.MAX_VALUE);
	    }

	    /**
	     * Evaluates the continued fraction at the value x.
	     * @param x the evaluation point.
	     * @param epsilon maximum error allowed.
	     * @return the value of the continued fraction evaluated at x. 
	     * @throws MathException if the algorithm fails to converge.
	     */
	    public double evaluate(double x, double epsilon) {
	        return evaluate(x, epsilon, Integer.MAX_VALUE);
	    }

	    /**
	     * Evaluates the continued fraction at the value x.
	     * @param x the evaluation point.
	     * @param maxIterations maximum number of convergents
	     * @return the value of the continued fraction evaluated at x. 
	     * @throws MathException if the algorithm fails to converge.
	     */
	    public double evaluate(double x, int maxIterations) {
	        return evaluate(x, DEFAULT_EPSILON, maxIterations);
	    }

	    /**
	     * Evaluates the continued fraction at the value x.
	     * 
	     * The implementation of this method is based on:
	     * <ul>
	     * <li>O. E-gecio-glu, C . K. Koc, J. Rifa i Coma,
	     * <a href="http://citeseer.ist.psu.edu/egecioglu91fast.html">
	     * On Fast Computation of Continued Fractions</a>, Computers Math. Applic.,
	     * 21(2--3), 1991, 167--169.</li>
	     * </ul>
	     * 
	     * @param x the evaluation point.
	     * @param epsilon maximum error allowed.
	     * @param maxIterations maximum number of convergents
	     * @return the value of the continued fraction evaluated at x. 
	     * @throws MathException if the algorithm fails to converge.
	     */
	    public double evaluate(double x, double epsilon, int maxIterations)
	    {
	        double[][] f = new double[2][2];
	        double[][] a = new double[2][2];
	        double[][] an = new double[2][2];

	        a[0][0] = getA(0, x);
	        a[0][1] = 1.0;
	        a[1][0] = 1.0;
	        a[1][1] = 0.0;

	        return evaluate(1, x, a, an, f, epsilon, maxIterations);
	    }

	    /**
	     * Evaluates the n-th convergent, fn = pn / qn, for this continued fraction
	     * at the value x.
	     * @param n the convergent to compute.
	     * @param x the evaluation point.
	     * @param a (n-1)-th convergent matrix.  (Input)
	     * @param an the n-th coefficient matrix. (Output)
	     * @param f the n-th convergent matrix. (Output)
	     * @param epsilon maximum error allowed.
	     * @param maxIterations maximum number of convergents
	     * @return the value of the the n-th convergent for this continued fraction
	     *         evaluated at x. 
	     * @throws MathException if the algorithm fails to converge.
	     */
	    private double evaluate(
	        int n,
	        double x,
	        double[][] a,
	        double[][] an,
	        double[][] f,
	        double epsilon,
	        int maxIterations) 
	    {
	        double ret;

	        // create next matrix
	        an[0][0] = getA(n, x);
	        an[0][1] = 1.0;
	        an[1][0] = getB(n, x);
	        an[1][1] = 0.0;

	        // multiply a and an, save as f
	        f[0][0] = (a[0][0] * an[0][0]) + (a[0][1] * an[1][0]);
	        f[0][1] = (a[0][0] * an[0][1]) + (a[0][1] * an[1][1]);
	        f[1][0] = (a[1][0] * an[0][0]) + (a[1][1] * an[1][0]);
	        f[1][1] = (a[1][0] * an[0][1]) + (a[1][1] * an[1][1]);

	        // determine if we're close enough
	        if (Math.abs((f[0][0] * f[1][1]) - (f[1][0] * f[0][1])) <
	            Math.abs(epsilon * f[1][0] * f[1][1]))
	        {
	            ret = f[0][0] / f[1][0];
	        } else {
	            if (n >= maxIterations) {
	                throw new RuntimeException(
	                    "Continued fraction convergents failed to converge.");
	            }
	            // compute next
	            ret = evaluate(n + 1, x, f /* new a */
	            , an /* reuse an */
	            , a /* new f */
	            , epsilon, maxIterations);
	        }

	        return ret;
	    }
	}
	
    /** Maximum allowed numerical error. */
    private static final double DEFAULT_EPSILON = 10e-9;

    /** Lanczos coefficients */
    private static double[] lanczos =
    {
        0.99999999999999709182,
        57.156235665862923517,
        -59.597960355475491248,
        14.136097974741747174,
        -0.49191381609762019978,
        .33994649984811888699e-4,
        .46523628927048575665e-4,
        -.98374475304879564677e-4,
        .15808870322491248884e-3,
        -.21026444172410488319e-3,
        .21743961811521264320e-3,
        -.16431810653676389022e-3,
        .84418223983852743293e-4,
        -.26190838401581408670e-4,
        .36899182659531622704e-5,
    };

    
    /**
     * Returns the natural logarithm of the gamma function &#915;(x).
     *
     * The implementation of this method is based on:
     * <ul>
     * <li><a href="http://mathworld.wolfram.com/GammaFunction.html">
     * Gamma Function</a>, equation (28).</li>
     * <li><a href="http://mathworld.wolfram.com/LanczosApproximation.html">
     * Lanczos Approximation</a>, equations (1) through (5).</li>
     * <li><a href="http://my.fit.edu/~gabdo/gamma.txt">Paul Godfrey, A note on
     * the computation of the convergent Lanczos complex Gamma approximation
     * </a></li>
     * </ul>
     * 
     * @param x the value.
     * @return log(&#915;(x))
     */
    public static double logGamma(double x) {
        double ret;

        if (Double.isNaN(x) || (x <= 0.0)) {
            ret = Double.NaN;
        } else {
            double g = 607.0 / 128.0;

            double sum = 0.0;
            for (int i = 1; i < lanczos.length; ++i) {
                sum = sum + (lanczos[i] / (x + i));
            }
            sum = sum + lanczos[0];

            double tmp = x + g + .5;
            ret = ((x + .5) * Math.log(tmp)) - tmp +
                (.5 * Math.log(2.0 * Math.PI)) + Math.log(sum) - Math.log(x);
        }

        return ret;
    }

    /**
     * Returns the regularized gamma function P(a, x).
     * 
     * @param a the a parameter.
     * @param x the value.
     * @return the regularized gamma function P(a, x)
     * @throws MathException if the algorithm fails to converge.
     */
    public static double regularizedGammaP(double a, double x) {
        return regularizedGammaP(a, x, DEFAULT_EPSILON, Integer.MAX_VALUE);
    }
        
        
    /**
     * Returns the regularized gamma function P(a, x).
     * 
     * The implementation of this method is based on:
     * <ul>
     * <li>
     * <a href="http://mathworld.wolfram.com/RegularizedGammaFunction.html">
     * Regularized Gamma Function</a>, equation (1).</li>
     * <li>
     * <a href="http://mathworld.wolfram.com/IncompleteGammaFunction.html">
     * Incomplete Gamma Function</a>, equation (4).</li>
     * <li>
     * <a href="http://mathworld.wolfram.com/ConfluentHypergeometricFunctionoftheFirstKind.html">
     * Confluent Hypergeometric Function of the First Kind</a>, equation (1).
     * </li>
     * </ul>
     * 
     * @param a the a parameter.
     * @param x the value.
     * @param epsilon When the absolute value of the nth item in the
     *                series is less than epsilon the approximation ceases
     *                to calculate further elements in the series.
     * @param maxIterations Maximum number of "iterations" to complete. 
     * @return the regularized gamma function P(a, x)
     * @throws MathException if the algorithm fails to converge.
     */
    public static double regularizedGammaP(double a, double x, double epsilon, int maxIterations) {
        double ret;

        if (Double.isNaN(a) || Double.isNaN(x) || (a <= 0.0) || (x < 0.0)) {
            ret = Double.NaN;
        } else if (x == 0.0) {
            ret = 0.0;
        } else if (a > 1.0 && x > a) {
            // use regularizedGammaQ because it should converge faster in this
            // case.
            ret = 1.0 - regularizedGammaQ(a, x, epsilon, maxIterations);
        } else {
            // calculate series
            double n = 0.0; // current element index
            double an = 1.0 / a; // n-th element in the series
            double sum = an; // partial sum
            while (Math.abs(an) > epsilon && n < maxIterations) {
                // compute next element in the series
                n = n + 1.0;
                an = an * (x / (a + n));

                // update partial sum
                sum = sum + an;
            }
            if (n >= maxIterations) {
                throw new RuntimeException(
                    "maximum number of iterations reached");
            } else {
                ret = Math.exp(-x + (a * Math.log(x)) - logGamma(a)) * sum;
            }
        }

        return ret;
    }
    
    /**
     * Returns the regularized gamma function Q(a, x) = 1 - P(a, x).
     * 
     * @param a the a parameter.
     * @param x the value.
     * @return the regularized gamma function Q(a, x)
     * @throws MathException if the algorithm fails to converge.
     */
    public static double regularizedGammaQ(double a, double x) {
        return regularizedGammaQ(a, x, DEFAULT_EPSILON, Integer.MAX_VALUE);
    }
    
    /**
     * Returns the regularized gamma function Q(a, x) = 1 - P(a, x).
     * 
     * The implementation of this method is based on:
     * <ul>
     * <li>
     * <a href="http://mathworld.wolfram.com/RegularizedGammaFunction.html">
     * Regularized Gamma Function</a>, equation (1).</li>
     * <li>
     * <a href="    http://functions.wolfram.com/GammaBetaErf/GammaRegularized/10/0003/">
     * Regularized incomplete gamma function: Continued fraction representations  (formula 06.08.10.0003)</a></li>
     * </ul>
     * 
     * @param a the a parameter.
     * @param x the value.
     * @param epsilon When the absolute value of the nth item in the
     *                series is less than epsilon the approximation ceases
     *                to calculate further elements in the series.
     * @param maxIterations Maximum number of "iterations" to complete. 
     * @return the regularized gamma function P(a, x)
     * @throws MathException if the algorithm fails to converge.
     */
	public static double regularizedGammaQ(final double a, 
                                           double x, 
                                           double epsilon, 
                                           int maxIterations) 
    {
        double ret;

        if (Double.isNaN(a) || Double.isNaN(x) || (a <= 0.0) || (x < 0.0)) {
            ret = Double.NaN;
        } else if (x == 0.0) {
            ret = 1.0;
        } else if (x < a || a <= 1.0) {
            // use regularizedGammaP because it should converge faster in this
            // case.
            ret = 1.0 - regularizedGammaP(a, x, epsilon, maxIterations);
        } else {
            // create continued fraction
            ContinuedFraction cf = new ContinuedFraction() {
                protected double getA(int n, double x) {
                    return ((2.0 * n) + 1.0) - a + x;
                }

                protected double getB(int n, double x) {
                    return n * (a - n);
                }
            };
            
            ret = 1.0 / cf.evaluate(x, epsilon, maxIterations);
            ret = Math.exp(-x + (a * Math.log(x)) - logGamma(a)) * ret;
        }

        return ret;
    }
	
	/**
     * Returns the error function erf(x).
     * 
     * The implementation of this method is based on:
     * <ul>
     * <li>
     * <a href="http://mathworld.wolfram.com/Erf.html">
     * Erf</a>, equation (3).</li>
     * </ul>
     * 
     * @param x the value.
     * @return the error function erf(x)
     * @throws MathException if the algorithm fails to converge.
     */
    public static double erf(double x) {
    	if (Double.isInfinite(x)) {
    		if (x>0)
    			return 1;
    		else
    			return -1;
    	}
    	if (Double.isNaN(x))
    		return Double.NaN;
        double ret = regularizedGammaP(0.5, x * x, 1.0e-15, 10000);
        if (x < 0) {
            ret = -ret;
        }
        return ret;
    }
}
