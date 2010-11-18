; Auto-generated. Do not edit!


(in-package comp_ehow-srv)


;//! \htmlinclude EhowToOWL-request.msg.html

(defclass <EhowToOWL-request> (ros-message)
  ((command
    :reader command-val
    :initarg :command
    :type string
    :initform ""))
)
(defmethod serialize ((msg <EhowToOWL-request>) ostream)
  "Serializes a message object of type '<EhowToOWL-request>"
  (let ((__ros_str_len (length (slot-value msg 'command))))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) (slot-value msg 'command))
)
(defmethod deserialize ((msg <EhowToOWL-request>) istream)
  "Deserializes a message object of type '<EhowToOWL-request>"
  (let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (slot-value msg 'command) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (slot-value msg 'command) __ros_str_idx) (code-char (read-byte istream)))))
  msg
)
(defmethod ros-datatype ((msg (eql '<EhowToOWL-request>)))
  "Returns string type for a service object of type '<EhowToOWL-request>"
  "comp_ehow/EhowToOWLRequest")
(defmethod md5sum ((type (eql '<EhowToOWL-request>)))
  "Returns md5sum for a message object of type '<EhowToOWL-request>"
  "757be2a3a997cc31d4d3125810ee07f5")
(defmethod message-definition ((type (eql '<EhowToOWL-request>)))
  "Returns full string definition for message of type '<EhowToOWL-request>"
  (format nil "# Moritz Tenorth, tenorth@cs.tum.edu~%# service to retrieve a how-to for a query string and return an OWL representation~%string command~%~%"))
(defmethod serialization-length ((msg <EhowToOWL-request>))
  (+ 0
     4 (length (slot-value msg 'command))
))
(defmethod ros-message-to-list ((msg <EhowToOWL-request>))
  "Converts a ROS message object to a list"
  (list '<EhowToOWL-request>
    (cons ':command (command-val msg))
))
;//! \htmlinclude EhowToOWL-response.msg.html

(defclass <EhowToOWL-response> (ros-message)
  ((owl_instructions
    :reader owl_instructions-val
    :initarg :owl_instructions
    :type string
    :initform ""))
)
(defmethod serialize ((msg <EhowToOWL-response>) ostream)
  "Serializes a message object of type '<EhowToOWL-response>"
  (let ((__ros_str_len (length (slot-value msg 'owl_instructions))))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) (slot-value msg 'owl_instructions))
)
(defmethod deserialize ((msg <EhowToOWL-response>) istream)
  "Deserializes a message object of type '<EhowToOWL-response>"
  (let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (slot-value msg 'owl_instructions) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (slot-value msg 'owl_instructions) __ros_str_idx) (code-char (read-byte istream)))))
  msg
)
(defmethod ros-datatype ((msg (eql '<EhowToOWL-response>)))
  "Returns string type for a service object of type '<EhowToOWL-response>"
  "comp_ehow/EhowToOWLResponse")
(defmethod md5sum ((type (eql '<EhowToOWL-response>)))
  "Returns md5sum for a message object of type '<EhowToOWL-response>"
  "757be2a3a997cc31d4d3125810ee07f5")
(defmethod message-definition ((type (eql '<EhowToOWL-response>)))
  "Returns full string definition for message of type '<EhowToOWL-response>"
  (format nil "string owl_instructions~%~%~%"))
(defmethod serialization-length ((msg <EhowToOWL-response>))
  (+ 0
     4 (length (slot-value msg 'owl_instructions))
))
(defmethod ros-message-to-list ((msg <EhowToOWL-response>))
  "Converts a ROS message object to a list"
  (list '<EhowToOWL-response>
    (cons ':owl_instructions (owl_instructions-val msg))
))
(defmethod service-request-type ((msg (eql 'EhowToOWL)))
  '<EhowToOWL-request>)
(defmethod service-response-type ((msg (eql 'EhowToOWL)))
  '<EhowToOWL-response>)
(defmethod ros-datatype ((msg (eql 'EhowToOWL)))
  "Returns string type for a service object of type '<EhowToOWL>"
  "comp_ehow/EhowToOWL")
