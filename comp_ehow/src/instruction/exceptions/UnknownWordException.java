package instruction.exceptions;

public class UnknownWordException extends Exception {


	private static final long serialVersionUID = -4039575417793332735L;

	public UnknownWordException(String msg) {
		super(msg);
	}
}
