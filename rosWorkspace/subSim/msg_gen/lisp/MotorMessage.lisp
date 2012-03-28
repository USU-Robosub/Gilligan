; Auto-generated. Do not edit!


(cl:in-package subSim-msg)


;//! \htmlinclude MotorMessage.msg.html

(cl:defclass <MotorMessage> (roslisp-msg-protocol:ros-message)
  ((mask
    :reader mask
    :initarg :mask
    :type cl:fixnum
    :initform 0)
   (Left
    :reader Left
    :initarg :Left
    :type cl:fixnum
    :initform 0)
   (Right
    :reader Right
    :initarg :Right
    :type cl:fixnum
    :initform 0)
   (FrontDepth
    :reader FrontDepth
    :initarg :FrontDepth
    :type cl:fixnum
    :initform 0)
   (RearDepth
    :reader RearDepth
    :initarg :RearDepth
    :type cl:fixnum
    :initform 0)
   (FrontTurn
    :reader FrontTurn
    :initarg :FrontTurn
    :type cl:fixnum
    :initform 0)
   (RearTurn
    :reader RearTurn
    :initarg :RearTurn
    :type cl:fixnum
    :initform 0))
)

(cl:defclass MotorMessage (<MotorMessage>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MotorMessage>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MotorMessage)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name subSim-msg:<MotorMessage> is deprecated: use subSim-msg:MotorMessage instead.")))

(cl:ensure-generic-function 'mask-val :lambda-list '(m))
(cl:defmethod mask-val ((m <MotorMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader subSim-msg:mask-val is deprecated.  Use subSim-msg:mask instead.")
  (mask m))

(cl:ensure-generic-function 'Left-val :lambda-list '(m))
(cl:defmethod Left-val ((m <MotorMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader subSim-msg:Left-val is deprecated.  Use subSim-msg:Left instead.")
  (Left m))

(cl:ensure-generic-function 'Right-val :lambda-list '(m))
(cl:defmethod Right-val ((m <MotorMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader subSim-msg:Right-val is deprecated.  Use subSim-msg:Right instead.")
  (Right m))

(cl:ensure-generic-function 'FrontDepth-val :lambda-list '(m))
(cl:defmethod FrontDepth-val ((m <MotorMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader subSim-msg:FrontDepth-val is deprecated.  Use subSim-msg:FrontDepth instead.")
  (FrontDepth m))

(cl:ensure-generic-function 'RearDepth-val :lambda-list '(m))
(cl:defmethod RearDepth-val ((m <MotorMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader subSim-msg:RearDepth-val is deprecated.  Use subSim-msg:RearDepth instead.")
  (RearDepth m))

(cl:ensure-generic-function 'FrontTurn-val :lambda-list '(m))
(cl:defmethod FrontTurn-val ((m <MotorMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader subSim-msg:FrontTurn-val is deprecated.  Use subSim-msg:FrontTurn instead.")
  (FrontTurn m))

(cl:ensure-generic-function 'RearTurn-val :lambda-list '(m))
(cl:defmethod RearTurn-val ((m <MotorMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader subSim-msg:RearTurn-val is deprecated.  Use subSim-msg:RearTurn instead.")
  (RearTurn m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MotorMessage>) ostream)
  "Serializes a message object of type '<MotorMessage>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mask)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'Left)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'Right)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'FrontDepth)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'RearDepth)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'FrontTurn)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'RearTurn)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MotorMessage>) istream)
  "Deserializes a message object of type '<MotorMessage>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mask)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Left) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Right) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'FrontDepth) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'RearDepth) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'FrontTurn) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'RearTurn) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MotorMessage>)))
  "Returns string type for a message object of type '<MotorMessage>"
  "subSim/MotorMessage")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MotorMessage)))
  "Returns string type for a message object of type 'MotorMessage"
  "subSim/MotorMessage")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MotorMessage>)))
  "Returns md5sum for a message object of type '<MotorMessage>"
  "eb586c06729a6d7d1e877d5edd5c857e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MotorMessage)))
  "Returns md5sum for a message object of type 'MotorMessage"
  "eb586c06729a6d7d1e877d5edd5c857e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MotorMessage>)))
  "Returns full string definition for message of type '<MotorMessage>"
  (cl:format cl:nil "uint8 mask~%~%int16 Left~%int16 Right~%~%int16 FrontDepth~%int16 RearDepth~%~%int16 FrontTurn~%int16 RearTurn~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MotorMessage)))
  "Returns full string definition for message of type 'MotorMessage"
  (cl:format cl:nil "uint8 mask~%~%int16 Left~%int16 Right~%~%int16 FrontDepth~%int16 RearDepth~%~%int16 FrontTurn~%int16 RearTurn~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MotorMessage>))
  (cl:+ 0
     1
     2
     2
     2
     2
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MotorMessage>))
  "Converts a ROS message object to a list"
  (cl:list 'MotorMessage
    (cl:cons ':mask (mask msg))
    (cl:cons ':Left (Left msg))
    (cl:cons ':Right (Right msg))
    (cl:cons ':FrontDepth (FrontDepth msg))
    (cl:cons ':RearDepth (RearDepth msg))
    (cl:cons ':FrontTurn (FrontTurn msg))
    (cl:cons ':RearTurn (RearTurn msg))
))
