; Auto-generated. Do not edit!


(cl:in-package move_robot-srv)


;//! \htmlinclude xycoord-request.msg.html

(cl:defclass <xycoord-request> (roslisp-msg-protocol:ros-message)
  ((occ_theta
    :reader occ_theta
    :initarg :occ_theta
    :type cl:float
    :initform 0.0)
   (l
    :reader l
    :initarg :l
    :type cl:float
    :initform 0.0)
   (w
    :reader w
    :initarg :w
    :type cl:float
    :initform 0.0))
)

(cl:defclass xycoord-request (<xycoord-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <xycoord-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'xycoord-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name move_robot-srv:<xycoord-request> is deprecated: use move_robot-srv:xycoord-request instead.")))

(cl:ensure-generic-function 'occ_theta-val :lambda-list '(m))
(cl:defmethod occ_theta-val ((m <xycoord-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader move_robot-srv:occ_theta-val is deprecated.  Use move_robot-srv:occ_theta instead.")
  (occ_theta m))

(cl:ensure-generic-function 'l-val :lambda-list '(m))
(cl:defmethod l-val ((m <xycoord-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader move_robot-srv:l-val is deprecated.  Use move_robot-srv:l instead.")
  (l m))

(cl:ensure-generic-function 'w-val :lambda-list '(m))
(cl:defmethod w-val ((m <xycoord-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader move_robot-srv:w-val is deprecated.  Use move_robot-srv:w instead.")
  (w m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <xycoord-request>) ostream)
  "Serializes a message object of type '<xycoord-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'occ_theta))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'l))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'w))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <xycoord-request>) istream)
  "Deserializes a message object of type '<xycoord-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'occ_theta) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'l) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'w) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<xycoord-request>)))
  "Returns string type for a service object of type '<xycoord-request>"
  "move_robot/xycoordRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'xycoord-request)))
  "Returns string type for a service object of type 'xycoord-request"
  "move_robot/xycoordRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<xycoord-request>)))
  "Returns md5sum for a message object of type '<xycoord-request>"
  "468f9ac078489289ae3d0296932d3837")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'xycoord-request)))
  "Returns md5sum for a message object of type 'xycoord-request"
  "468f9ac078489289ae3d0296932d3837")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<xycoord-request>)))
  "Returns full string definition for message of type '<xycoord-request>"
  (cl:format cl:nil "float64 occ_theta~%float64 l~%float64 w ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'xycoord-request)))
  "Returns full string definition for message of type 'xycoord-request"
  (cl:format cl:nil "float64 occ_theta~%float64 l~%float64 w ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <xycoord-request>))
  (cl:+ 0
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <xycoord-request>))
  "Converts a ROS message object to a list"
  (cl:list 'xycoord-request
    (cl:cons ':occ_theta (occ_theta msg))
    (cl:cons ':l (l msg))
    (cl:cons ':w (w msg))
))
;//! \htmlinclude xycoord-response.msg.html

(cl:defclass <xycoord-response> (roslisp-msg-protocol:ros-message)
  ((x2
    :reader x2
    :initarg :x2
    :type cl:float
    :initform 0.0)
   (y2
    :reader y2
    :initarg :y2
    :type cl:float
    :initform 0.0))
)

(cl:defclass xycoord-response (<xycoord-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <xycoord-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'xycoord-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name move_robot-srv:<xycoord-response> is deprecated: use move_robot-srv:xycoord-response instead.")))

(cl:ensure-generic-function 'x2-val :lambda-list '(m))
(cl:defmethod x2-val ((m <xycoord-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader move_robot-srv:x2-val is deprecated.  Use move_robot-srv:x2 instead.")
  (x2 m))

(cl:ensure-generic-function 'y2-val :lambda-list '(m))
(cl:defmethod y2-val ((m <xycoord-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader move_robot-srv:y2-val is deprecated.  Use move_robot-srv:y2 instead.")
  (y2 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <xycoord-response>) ostream)
  "Serializes a message object of type '<xycoord-response>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'x2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'y2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <xycoord-response>) istream)
  "Deserializes a message object of type '<xycoord-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x2) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y2) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<xycoord-response>)))
  "Returns string type for a service object of type '<xycoord-response>"
  "move_robot/xycoordResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'xycoord-response)))
  "Returns string type for a service object of type 'xycoord-response"
  "move_robot/xycoordResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<xycoord-response>)))
  "Returns md5sum for a message object of type '<xycoord-response>"
  "468f9ac078489289ae3d0296932d3837")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'xycoord-response)))
  "Returns md5sum for a message object of type 'xycoord-response"
  "468f9ac078489289ae3d0296932d3837")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<xycoord-response>)))
  "Returns full string definition for message of type '<xycoord-response>"
  (cl:format cl:nil "float64 x2~%float64 y2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'xycoord-response)))
  "Returns full string definition for message of type 'xycoord-response"
  (cl:format cl:nil "float64 x2~%float64 y2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <xycoord-response>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <xycoord-response>))
  "Converts a ROS message object to a list"
  (cl:list 'xycoord-response
    (cl:cons ':x2 (x2 msg))
    (cl:cons ':y2 (y2 msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'xycoord)))
  'xycoord-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'xycoord)))
  'xycoord-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'xycoord)))
  "Returns string type for a service object of type '<xycoord>"
  "move_robot/xycoord")