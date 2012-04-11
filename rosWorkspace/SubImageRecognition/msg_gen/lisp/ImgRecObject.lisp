; Auto-generated. Do not edit!


(cl:in-package SubImageRecognition-msg)


;//! \htmlinclude ImgRecObject.msg.html

(cl:defclass <ImgRecObject> (roslisp-msg-protocol:ros-message)
  ((stamp
    :reader stamp
    :initarg :stamp
    :type cl:real
    :initform 0)
   (name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
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
   (height
    :reader height
    :initarg :height
    :type cl:fixnum
    :initform 0)
   (width
    :reader width
    :initarg :width
    :type cl:fixnum
    :initform 0)
   (confidence
    :reader confidence
    :initarg :confidence
    :type cl:float
    :initform 0.0))
)

(cl:defclass ImgRecObject (<ImgRecObject>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ImgRecObject>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ImgRecObject)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name SubImageRecognition-msg:<ImgRecObject> is deprecated: use SubImageRecognition-msg:ImgRecObject instead.")))

(cl:ensure-generic-function 'stamp-val :lambda-list '(m))
(cl:defmethod stamp-val ((m <ImgRecObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader SubImageRecognition-msg:stamp-val is deprecated.  Use SubImageRecognition-msg:stamp instead.")
  (stamp m))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <ImgRecObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader SubImageRecognition-msg:name-val is deprecated.  Use SubImageRecognition-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'center_x-val :lambda-list '(m))
(cl:defmethod center_x-val ((m <ImgRecObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader SubImageRecognition-msg:center_x-val is deprecated.  Use SubImageRecognition-msg:center_x instead.")
  (center_x m))

(cl:ensure-generic-function 'center_y-val :lambda-list '(m))
(cl:defmethod center_y-val ((m <ImgRecObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader SubImageRecognition-msg:center_y-val is deprecated.  Use SubImageRecognition-msg:center_y instead.")
  (center_y m))

(cl:ensure-generic-function 'rotation-val :lambda-list '(m))
(cl:defmethod rotation-val ((m <ImgRecObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader SubImageRecognition-msg:rotation-val is deprecated.  Use SubImageRecognition-msg:rotation instead.")
  (rotation m))

(cl:ensure-generic-function 'height-val :lambda-list '(m))
(cl:defmethod height-val ((m <ImgRecObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader SubImageRecognition-msg:height-val is deprecated.  Use SubImageRecognition-msg:height instead.")
  (height m))

(cl:ensure-generic-function 'width-val :lambda-list '(m))
(cl:defmethod width-val ((m <ImgRecObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader SubImageRecognition-msg:width-val is deprecated.  Use SubImageRecognition-msg:width instead.")
  (width m))

(cl:ensure-generic-function 'confidence-val :lambda-list '(m))
(cl:defmethod confidence-val ((m <ImgRecObject>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader SubImageRecognition-msg:confidence-val is deprecated.  Use SubImageRecognition-msg:confidence instead.")
  (confidence m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ImgRecObject>) ostream)
  "Serializes a message object of type '<ImgRecObject>"
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
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'center_x)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'center_x)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'center_y)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'center_y)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rotation))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'height)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'width)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'width)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'confidence))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ImgRecObject>) istream)
  "Deserializes a message object of type '<ImgRecObject>"
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
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
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
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'height)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'width)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'confidence) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ImgRecObject>)))
  "Returns string type for a message object of type '<ImgRecObject>"
  "SubImageRecognition/ImgRecObject")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ImgRecObject)))
  "Returns string type for a message object of type 'ImgRecObject"
  "SubImageRecognition/ImgRecObject")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ImgRecObject>)))
  "Returns md5sum for a message object of type '<ImgRecObject>"
  "e457d95566509e9119cff64bd5619f85")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ImgRecObject)))
  "Returns md5sum for a message object of type 'ImgRecObject"
  "e457d95566509e9119cff64bd5619f85")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ImgRecObject>)))
  "Returns full string definition for message of type '<ImgRecObject>"
  (cl:format cl:nil "time stamp~%string name~%uint16 center_x~%uint16 center_y~%float32 rotation~%uint16 height~%uint16 width~%float32 confidence~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ImgRecObject)))
  "Returns full string definition for message of type 'ImgRecObject"
  (cl:format cl:nil "time stamp~%string name~%uint16 center_x~%uint16 center_y~%float32 rotation~%uint16 height~%uint16 width~%float32 confidence~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ImgRecObject>))
  (cl:+ 0
     8
     4 (cl:length (cl:slot-value msg 'name))
     2
     2
     4
     2
     2
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ImgRecObject>))
  "Converts a ROS message object to a list"
  (cl:list 'ImgRecObject
    (cl:cons ':stamp (stamp msg))
    (cl:cons ':name (name msg))
    (cl:cons ':center_x (center_x msg))
    (cl:cons ':center_y (center_y msg))
    (cl:cons ':rotation (rotation msg))
    (cl:cons ':height (height msg))
    (cl:cons ':width (width msg))
    (cl:cons ':confidence (confidence msg))
))
