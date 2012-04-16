; Auto-generated. Do not edit!


(cl:in-package SubImageRecognition-srv)


;//! \htmlinclude ListAlgorithms-request.msg.html

(cl:defclass <ListAlgorithms-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ListAlgorithms-request (<ListAlgorithms-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ListAlgorithms-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ListAlgorithms-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name SubImageRecognition-srv:<ListAlgorithms-request> is deprecated: use SubImageRecognition-srv:ListAlgorithms-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ListAlgorithms-request>) ostream)
  "Serializes a message object of type '<ListAlgorithms-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ListAlgorithms-request>) istream)
  "Deserializes a message object of type '<ListAlgorithms-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ListAlgorithms-request>)))
  "Returns string type for a service object of type '<ListAlgorithms-request>"
  "SubImageRecognition/ListAlgorithmsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ListAlgorithms-request)))
  "Returns string type for a service object of type 'ListAlgorithms-request"
  "SubImageRecognition/ListAlgorithmsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ListAlgorithms-request>)))
  "Returns md5sum for a message object of type '<ListAlgorithms-request>"
  "5c271c164803b5cf2105bc913a23f17a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ListAlgorithms-request)))
  "Returns md5sum for a message object of type 'ListAlgorithms-request"
  "5c271c164803b5cf2105bc913a23f17a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ListAlgorithms-request>)))
  "Returns full string definition for message of type '<ListAlgorithms-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ListAlgorithms-request)))
  "Returns full string definition for message of type 'ListAlgorithms-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ListAlgorithms-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ListAlgorithms-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ListAlgorithms-request
))
;//! \htmlinclude ListAlgorithms-response.msg.html

(cl:defclass <ListAlgorithms-response> (roslisp-msg-protocol:ros-message)
  ((algorithms
    :reader algorithms
    :initarg :algorithms
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass ListAlgorithms-response (<ListAlgorithms-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ListAlgorithms-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ListAlgorithms-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name SubImageRecognition-srv:<ListAlgorithms-response> is deprecated: use SubImageRecognition-srv:ListAlgorithms-response instead.")))

(cl:ensure-generic-function 'algorithms-val :lambda-list '(m))
(cl:defmethod algorithms-val ((m <ListAlgorithms-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader SubImageRecognition-srv:algorithms-val is deprecated.  Use SubImageRecognition-srv:algorithms instead.")
  (algorithms m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ListAlgorithms-response>) ostream)
  "Serializes a message object of type '<ListAlgorithms-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'algorithms))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'algorithms))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ListAlgorithms-response>) istream)
  "Deserializes a message object of type '<ListAlgorithms-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'algorithms) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'algorithms)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ListAlgorithms-response>)))
  "Returns string type for a service object of type '<ListAlgorithms-response>"
  "SubImageRecognition/ListAlgorithmsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ListAlgorithms-response)))
  "Returns string type for a service object of type 'ListAlgorithms-response"
  "SubImageRecognition/ListAlgorithmsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ListAlgorithms-response>)))
  "Returns md5sum for a message object of type '<ListAlgorithms-response>"
  "5c271c164803b5cf2105bc913a23f17a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ListAlgorithms-response)))
  "Returns md5sum for a message object of type 'ListAlgorithms-response"
  "5c271c164803b5cf2105bc913a23f17a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ListAlgorithms-response>)))
  "Returns full string definition for message of type '<ListAlgorithms-response>"
  (cl:format cl:nil "string[] algorithms~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ListAlgorithms-response)))
  "Returns full string definition for message of type 'ListAlgorithms-response"
  (cl:format cl:nil "string[] algorithms~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ListAlgorithms-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'algorithms) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ListAlgorithms-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ListAlgorithms-response
    (cl:cons ':algorithms (algorithms msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ListAlgorithms)))
  'ListAlgorithms-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ListAlgorithms)))
  'ListAlgorithms-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ListAlgorithms)))
  "Returns string type for a service object of type '<ListAlgorithms>"
  "SubImageRecognition/ListAlgorithms")
