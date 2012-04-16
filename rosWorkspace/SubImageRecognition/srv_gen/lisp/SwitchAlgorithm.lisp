; Auto-generated. Do not edit!


(cl:in-package SubImageRecognition-srv)


;//! \htmlinclude SwitchAlgorithm-request.msg.html

(cl:defclass <SwitchAlgorithm-request> (roslisp-msg-protocol:ros-message)
  ((algorithm
    :reader algorithm
    :initarg :algorithm
    :type cl:string
    :initform "")
   (enabled
    :reader enabled
    :initarg :enabled
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SwitchAlgorithm-request (<SwitchAlgorithm-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SwitchAlgorithm-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SwitchAlgorithm-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name SubImageRecognition-srv:<SwitchAlgorithm-request> is deprecated: use SubImageRecognition-srv:SwitchAlgorithm-request instead.")))

(cl:ensure-generic-function 'algorithm-val :lambda-list '(m))
(cl:defmethod algorithm-val ((m <SwitchAlgorithm-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader SubImageRecognition-srv:algorithm-val is deprecated.  Use SubImageRecognition-srv:algorithm instead.")
  (algorithm m))

(cl:ensure-generic-function 'enabled-val :lambda-list '(m))
(cl:defmethod enabled-val ((m <SwitchAlgorithm-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader SubImageRecognition-srv:enabled-val is deprecated.  Use SubImageRecognition-srv:enabled instead.")
  (enabled m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SwitchAlgorithm-request>) ostream)
  "Serializes a message object of type '<SwitchAlgorithm-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'algorithm))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'algorithm))
  (cl:let* ((signed (cl:slot-value msg 'enabled)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SwitchAlgorithm-request>) istream)
  "Deserializes a message object of type '<SwitchAlgorithm-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'algorithm) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'algorithm) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'enabled) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SwitchAlgorithm-request>)))
  "Returns string type for a service object of type '<SwitchAlgorithm-request>"
  "SubImageRecognition/SwitchAlgorithmRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SwitchAlgorithm-request)))
  "Returns string type for a service object of type 'SwitchAlgorithm-request"
  "SubImageRecognition/SwitchAlgorithmRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SwitchAlgorithm-request>)))
  "Returns md5sum for a message object of type '<SwitchAlgorithm-request>"
  "5d33cd0d6349093380d55a2733f4e514")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SwitchAlgorithm-request)))
  "Returns md5sum for a message object of type 'SwitchAlgorithm-request"
  "5d33cd0d6349093380d55a2733f4e514")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SwitchAlgorithm-request>)))
  "Returns full string definition for message of type '<SwitchAlgorithm-request>"
  (cl:format cl:nil "string algorithm~%int8 enabled~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SwitchAlgorithm-request)))
  "Returns full string definition for message of type 'SwitchAlgorithm-request"
  (cl:format cl:nil "string algorithm~%int8 enabled~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SwitchAlgorithm-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'algorithm))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SwitchAlgorithm-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SwitchAlgorithm-request
    (cl:cons ':algorithm (algorithm msg))
    (cl:cons ':enabled (enabled msg))
))
;//! \htmlinclude SwitchAlgorithm-response.msg.html

(cl:defclass <SwitchAlgorithm-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SwitchAlgorithm-response (<SwitchAlgorithm-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SwitchAlgorithm-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SwitchAlgorithm-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name SubImageRecognition-srv:<SwitchAlgorithm-response> is deprecated: use SubImageRecognition-srv:SwitchAlgorithm-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <SwitchAlgorithm-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader SubImageRecognition-srv:result-val is deprecated.  Use SubImageRecognition-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SwitchAlgorithm-response>) ostream)
  "Serializes a message object of type '<SwitchAlgorithm-response>"
  (cl:let* ((signed (cl:slot-value msg 'result)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SwitchAlgorithm-response>) istream)
  "Deserializes a message object of type '<SwitchAlgorithm-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'result) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SwitchAlgorithm-response>)))
  "Returns string type for a service object of type '<SwitchAlgorithm-response>"
  "SubImageRecognition/SwitchAlgorithmResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SwitchAlgorithm-response)))
  "Returns string type for a service object of type 'SwitchAlgorithm-response"
  "SubImageRecognition/SwitchAlgorithmResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SwitchAlgorithm-response>)))
  "Returns md5sum for a message object of type '<SwitchAlgorithm-response>"
  "5d33cd0d6349093380d55a2733f4e514")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SwitchAlgorithm-response)))
  "Returns md5sum for a message object of type 'SwitchAlgorithm-response"
  "5d33cd0d6349093380d55a2733f4e514")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SwitchAlgorithm-response>)))
  "Returns full string definition for message of type '<SwitchAlgorithm-response>"
  (cl:format cl:nil "int8 result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SwitchAlgorithm-response)))
  "Returns full string definition for message of type 'SwitchAlgorithm-response"
  (cl:format cl:nil "int8 result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SwitchAlgorithm-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SwitchAlgorithm-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SwitchAlgorithm-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SwitchAlgorithm)))
  'SwitchAlgorithm-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SwitchAlgorithm)))
  'SwitchAlgorithm-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SwitchAlgorithm)))
  "Returns string type for a service object of type '<SwitchAlgorithm>"
  "SubImageRecognition/SwitchAlgorithm")
