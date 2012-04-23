; Auto-generated. Do not edit!


(cl:in-package GateTask-srv)


;//! \htmlinclude Toggle-request.msg.html

(cl:defclass <Toggle-request> (roslisp-msg-protocol:ros-message)
  ((enabled
    :reader enabled
    :initarg :enabled
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Toggle-request (<Toggle-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Toggle-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Toggle-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name GateTask-srv:<Toggle-request> is deprecated: use GateTask-srv:Toggle-request instead.")))

(cl:ensure-generic-function 'enabled-val :lambda-list '(m))
(cl:defmethod enabled-val ((m <Toggle-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader GateTask-srv:enabled-val is deprecated.  Use GateTask-srv:enabled instead.")
  (enabled m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Toggle-request>) ostream)
  "Serializes a message object of type '<Toggle-request>"
  (cl:let* ((signed (cl:slot-value msg 'enabled)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Toggle-request>) istream)
  "Deserializes a message object of type '<Toggle-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'enabled) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Toggle-request>)))
  "Returns string type for a service object of type '<Toggle-request>"
  "GateTask/ToggleRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Toggle-request)))
  "Returns string type for a service object of type 'Toggle-request"
  "GateTask/ToggleRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Toggle-request>)))
  "Returns md5sum for a message object of type '<Toggle-request>"
  "a2f3d572baaef05608a5c9b396bf270d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Toggle-request)))
  "Returns md5sum for a message object of type 'Toggle-request"
  "a2f3d572baaef05608a5c9b396bf270d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Toggle-request>)))
  "Returns full string definition for message of type '<Toggle-request>"
  (cl:format cl:nil "int8 enabled~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Toggle-request)))
  "Returns full string definition for message of type 'Toggle-request"
  (cl:format cl:nil "int8 enabled~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Toggle-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Toggle-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Toggle-request
    (cl:cons ':enabled (enabled msg))
))
;//! \htmlinclude Toggle-response.msg.html

(cl:defclass <Toggle-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Toggle-response (<Toggle-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Toggle-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Toggle-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name GateTask-srv:<Toggle-response> is deprecated: use GateTask-srv:Toggle-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <Toggle-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader GateTask-srv:result-val is deprecated.  Use GateTask-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Toggle-response>) ostream)
  "Serializes a message object of type '<Toggle-response>"
  (cl:let* ((signed (cl:slot-value msg 'result)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Toggle-response>) istream)
  "Deserializes a message object of type '<Toggle-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'result) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Toggle-response>)))
  "Returns string type for a service object of type '<Toggle-response>"
  "GateTask/ToggleResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Toggle-response)))
  "Returns string type for a service object of type 'Toggle-response"
  "GateTask/ToggleResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Toggle-response>)))
  "Returns md5sum for a message object of type '<Toggle-response>"
  "a2f3d572baaef05608a5c9b396bf270d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Toggle-response)))
  "Returns md5sum for a message object of type 'Toggle-response"
  "a2f3d572baaef05608a5c9b396bf270d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Toggle-response>)))
  "Returns full string definition for message of type '<Toggle-response>"
  (cl:format cl:nil "int8 result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Toggle-response)))
  "Returns full string definition for message of type 'Toggle-response"
  (cl:format cl:nil "int8 result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Toggle-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Toggle-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Toggle-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Toggle)))
  'Toggle-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Toggle)))
  'Toggle-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Toggle)))
  "Returns string type for a service object of type '<Toggle>"
  "GateTask/Toggle")
