; Auto-generated. Do not edit!


(cl:in-package SubImageRecognition-msg)


;//! \htmlinclude OrangeRectangle.msg.html

(cl:defclass <OrangeRectangle> (roslisp-msg-protocol:ros-message)
  ((stamp
    :reader stamp
    :initarg :stamp
    :type cl:real
    :initform 0)
   (center_x
    :reader center_x
    :initarg :center_x
    :type cl:fixnum
    :initform 0)
   (center_y
    :reader center_y
    :initarg :center_y
    :type cl:fixnum
    :initform 0)
   (rotation
    :reader rotation
    :initarg :rotation
    :type cl:float
    :initform 0.0)
   (confidence
    :reader confidence
    :initarg :confidence
    :type cl:float
    :initform 0.0))
)

(cl:defclass OrangeRectangle (<OrangeRectangle>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OrangeRectangle>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OrangeRectangle)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name SubImageRecognition-msg:<OrangeRectangle> is deprecated: use SubImageRecognition-msg:OrangeRectangle instead.")))

(cl:ensure-generic-function 'stamp-val :lambda-list '(m))
(cl:defmethod stamp-val ((m <OrangeRectangle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader SubImageRecognition-msg:stamp-val is deprecated.  Use SubImageRecognition-msg:stamp instead.")
  (stamp m))

(cl:ensure-generic-function 'center_x-val :lambda-list '(m))
(cl:defmethod center_x-val ((m <OrangeRectangle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader SubImageRecognition-msg:center_x-val is deprecated.  Use SubImageRecognition-msg:center_x instead.")
  (center_x m))

(cl:ensure-generic-function 'center_y-val :lambda-list '(m))
(cl:defmethod center_y-val ((m <OrangeRectangle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader SubImageRecognition-msg:center_y-val is deprecated.  Use SubImageRecognition-msg:center_y instead.")
  (center_y m))

(cl:ensure-generic-function 'rotation-val :lambda-list '(m))
(cl:defmethod rotation-val ((m <OrangeRectangle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader SubImageRecognition-msg:rotation-val is deprecated.  Use SubImageRecognition-msg:rotation instead.")
  (rotation m))

(cl:ensure-generic-function 'confidence-val :lambda-list '(m))
(cl:defmethod confidence-val ((m <OrangeRectangle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader SubImageRecognition-msg:confidence-val is deprecated.  Use SubImageRecognition-msg:confidence instead.")
  (confidence m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OrangeRectangle>) ostream)
  "Serializes a message object of type '<OrangeRectangle>"
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'stamp)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'stamp) (cl:floor (cl:slot-value msg 'stamp)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'center_x)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'center_x)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'center_y)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'center_y)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rotation))))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OrangeRectangle>) istream)
  "Deserializes a message object of type '<OrangeRectangle>"
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'stamp) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'center_x)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'center_x)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'center_y)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'center_y)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rotation) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'confidence) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OrangeRectangle>)))
  "Returns string type for a message object of type '<OrangeRectangle>"
  "SubImageRecognition/OrangeRectangle")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OrangeRectangle)))
  "Returns string type for a message object of type 'OrangeRectangle"
  "SubImageRecognition/OrangeRectangle")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OrangeRectangle>)))
  "Returns md5sum for a message object of type '<OrangeRectangle>"
  "f4d874cbf5602fc8af32db3aa825d47e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OrangeRectangle)))
  "Returns md5sum for a message object of type 'OrangeRectangle"
  "f4d874cbf5602fc8af32db3aa825d47e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OrangeRectangle>)))
  "Returns full string definition for message of type '<OrangeRectangle>"
  (cl:format cl:nil "time stamp~%uint16 center_x~%uint16 center_y~%float32 rotation~%float32 confidence~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OrangeRectangle)))
  "Returns full string definition for message of type 'OrangeRectangle"
  (cl:format cl:nil "time stamp~%uint16 center_x~%uint16 center_y~%float32 rotation~%float32 confidence~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OrangeRectangle>))
  (cl:+ 0
     8
     2
     2
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OrangeRectangle>))
  "Converts a ROS message object to a list"
  (cl:list 'OrangeRectangle
    (cl:cons ':stamp (stamp msg))
    (cl:cons ':center_x (center_x msg))
    (cl:cons ':center_y (center_y msg))
    (cl:cons ':rotation (rotation msg))
    (cl:cons ':confidence (confidence msg))
))
