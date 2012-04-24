; Auto-generated. Do not edit!


(cl:in-package NavigationControl-srv)


;//! \htmlinclude Navigate-request.msg.html

(cl:defclass <Navigate-request> (roslisp-msg-protocol:ros-message)
  ((camera_direction
    :reader camera_direction
    :initarg :camera_direction
    :type cl:fixnum
    :initform 0)
   (desired_x
    :reader desired_x
    :initarg :desired_x
    :type cl:fixnum
    :initform 0)
   (desired_y
    :reader desired_y
    :initarg :desired_y
    :type cl:fixnum
    :initform 0)
   (desired_rotation
    :reader desired_rotation
    :initarg :desired_rotation
    :type cl:float
    :initform 0.0))
)

(cl:defclass Navigate-request (<Navigate-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Navigate-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Navigate-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name NavigationControl-srv:<Navigate-request> is deprecated: use NavigationControl-srv:Navigate-request instead.")))

(cl:ensure-generic-function 'camera_direction-val :lambda-list '(m))
(cl:defmethod camera_direction-val ((m <Navigate-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader NavigationControl-srv:camera_direction-val is deprecated.  Use NavigationControl-srv:camera_direction instead.")
  (camera_direction m))

(cl:ensure-generic-function 'desired_x-val :lambda-list '(m))
(cl:defmethod desired_x-val ((m <Navigate-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader NavigationControl-srv:desired_x-val is deprecated.  Use NavigationControl-srv:desired_x instead.")
  (desired_x m))

(cl:ensure-generic-function 'desired_y-val :lambda-list '(m))
(cl:defmethod desired_y-val ((m <Navigate-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader NavigationControl-srv:desired_y-val is deprecated.  Use NavigationControl-srv:desired_y instead.")
  (desired_y m))

(cl:ensure-generic-function 'desired_rotation-val :lambda-list '(m))
(cl:defmethod desired_rotation-val ((m <Navigate-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader NavigationControl-srv:desired_rotation-val is deprecated.  Use NavigationControl-srv:desired_rotation instead.")
  (desired_rotation m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Navigate-request>) ostream)
  "Serializes a message object of type '<Navigate-request>"
  (cl:let* ((signed (cl:slot-value msg 'camera_direction)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'desired_x)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'desired_y)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'desired_rotation))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Navigate-request>) istream)
  "Deserializes a message object of type '<Navigate-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'camera_direction) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'desired_x) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'desired_y) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'desired_rotation) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Navigate-request>)))
  "Returns string type for a service object of type '<Navigate-request>"
  "NavigationControl/NavigateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Navigate-request)))
  "Returns string type for a service object of type 'Navigate-request"
  "NavigationControl/NavigateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Navigate-request>)))
  "Returns md5sum for a message object of type '<Navigate-request>"
  "256da788932aac90025db21f7d359b9e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Navigate-request)))
  "Returns md5sum for a message object of type 'Navigate-request"
  "256da788932aac90025db21f7d359b9e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Navigate-request>)))
  "Returns full string definition for message of type '<Navigate-request>"
  (cl:format cl:nil "int8 camera_direction~%int16 desired_x~%int16 desired_y~%float32 desired_rotation~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Navigate-request)))
  "Returns full string definition for message of type 'Navigate-request"
  (cl:format cl:nil "int8 camera_direction~%int16 desired_x~%int16 desired_y~%float32 desired_rotation~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Navigate-request>))
  (cl:+ 0
     1
     2
     2
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Navigate-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Navigate-request
    (cl:cons ':camera_direction (camera_direction msg))
    (cl:cons ':desired_x (desired_x msg))
    (cl:cons ':desired_y (desired_y msg))
    (cl:cons ':desired_rotation (desired_rotation msg))
))
;//! \htmlinclude Navigate-response.msg.html

(cl:defclass <Navigate-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Navigate-response (<Navigate-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Navigate-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Navigate-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name NavigationControl-srv:<Navigate-response> is deprecated: use NavigationControl-srv:Navigate-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <Navigate-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader NavigationControl-srv:result-val is deprecated.  Use NavigationControl-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Navigate-response>) ostream)
  "Serializes a message object of type '<Navigate-response>"
  (cl:let* ((signed (cl:slot-value msg 'result)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Navigate-response>) istream)
  "Deserializes a message object of type '<Navigate-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'result) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Navigate-response>)))
  "Returns string type for a service object of type '<Navigate-response>"
  "NavigationControl/NavigateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Navigate-response)))
  "Returns string type for a service object of type 'Navigate-response"
  "NavigationControl/NavigateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Navigate-response>)))
  "Returns md5sum for a message object of type '<Navigate-response>"
  "256da788932aac90025db21f7d359b9e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Navigate-response)))
  "Returns md5sum for a message object of type 'Navigate-response"
  "256da788932aac90025db21f7d359b9e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Navigate-response>)))
  "Returns full string definition for message of type '<Navigate-response>"
  (cl:format cl:nil "int8 result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Navigate-response)))
  "Returns full string definition for message of type 'Navigate-response"
  (cl:format cl:nil "int8 result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Navigate-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Navigate-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Navigate-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Navigate)))
  'Navigate-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Navigate)))
  'Navigate-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Navigate)))
  "Returns string type for a service object of type '<Navigate>"
  "NavigationControl/Navigate")
