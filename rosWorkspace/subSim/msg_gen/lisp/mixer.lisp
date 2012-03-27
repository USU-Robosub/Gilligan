; Auto-generated. Do not edit!


(cl:in-package subSim-msg)


;//! \htmlinclude mixer.msg.html

(cl:defclass <mixer> (roslisp-msg-protocol:ros-message)
  ((value
    :reader value
    :initarg :value
    :type cl:float
    :initform 0.0)
   (confidence
    :reader confidence
    :initarg :confidence
    :type cl:float
    :initform 0.0))
)

(cl:defclass mixer (<mixer>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <mixer>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'mixer)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name subSim-msg:<mixer> is deprecated: use subSim-msg:mixer instead.")))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <mixer>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader subSim-msg:value-val is deprecated.  Use subSim-msg:value instead.")
  (value m))

(cl:ensure-generic-function 'confidence-val :lambda-list '(m))
(cl:defmethod confidence-val ((m <mixer>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader subSim-msg:confidence-val is deprecated.  Use subSim-msg:confidence instead.")
  (confidence m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <mixer>) ostream)
  "Serializes a message object of type '<mixer>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'confidence))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <mixer>) istream)
  "Deserializes a message object of type '<mixer>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'value) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'confidence) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<mixer>)))
  "Returns string type for a message object of type '<mixer>"
  "subSim/mixer")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'mixer)))
  "Returns string type for a message object of type 'mixer"
  "subSim/mixer")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<mixer>)))
  "Returns md5sum for a message object of type '<mixer>"
  "ba840d81d8a0efbe22902b684b543995")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'mixer)))
  "Returns md5sum for a message object of type 'mixer"
  "ba840d81d8a0efbe22902b684b543995")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<mixer>)))
  "Returns full string definition for message of type '<mixer>"
  (cl:format cl:nil "float32 value~%float32 confidence~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'mixer)))
  "Returns full string definition for message of type 'mixer"
  (cl:format cl:nil "float32 value~%float32 confidence~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <mixer>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <mixer>))
  "Converts a ROS message object to a list"
  (cl:list 'mixer
    (cl:cons ':value (value msg))
    (cl:cons ':confidence (confidence msg))
))
