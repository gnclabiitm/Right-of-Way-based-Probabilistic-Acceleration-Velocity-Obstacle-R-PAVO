; Auto-generated. Do not edit!


(cl:in-package move_robot-srv)


;//! \htmlinclude sync_start-request.msg.html

(cl:defclass <sync_start-request> (roslisp-msg-protocol:ros-message)
  ((check_flag
    :reader check_flag
    :initarg :check_flag
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass sync_start-request (<sync_start-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <sync_start-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'sync_start-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name move_robot-srv:<sync_start-request> is deprecated: use move_robot-srv:sync_start-request instead.")))

(cl:ensure-generic-function 'check_flag-val :lambda-list '(m))
(cl:defmethod check_flag-val ((m <sync_start-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader move_robot-srv:check_flag-val is deprecated.  Use move_robot-srv:check_flag instead.")
  (check_flag m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <sync_start-request>) ostream)
  "Serializes a message object of type '<sync_start-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'check_flag) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <sync_start-request>) istream)
  "Deserializes a message object of type '<sync_start-request>"
    (cl:setf (cl:slot-value msg 'check_flag) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<sync_start-request>)))
  "Returns string type for a service object of type '<sync_start-request>"
  "move_robot/sync_startRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'sync_start-request)))
  "Returns string type for a service object of type 'sync_start-request"
  "move_robot/sync_startRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<sync_start-request>)))
  "Returns md5sum for a message object of type '<sync_start-request>"
  "f28b7d6543382a4046cd196e0567aca4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'sync_start-request)))
  "Returns md5sum for a message object of type 'sync_start-request"
  "f28b7d6543382a4046cd196e0567aca4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<sync_start-request>)))
  "Returns full string definition for message of type '<sync_start-request>"
  (cl:format cl:nil "bool check_flag~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'sync_start-request)))
  "Returns full string definition for message of type 'sync_start-request"
  (cl:format cl:nil "bool check_flag~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <sync_start-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <sync_start-request>))
  "Converts a ROS message object to a list"
  (cl:list 'sync_start-request
    (cl:cons ':check_flag (check_flag msg))
))
;//! \htmlinclude sync_start-response.msg.html

(cl:defclass <sync_start-response> (roslisp-msg-protocol:ros-message)
  ((set_flag
    :reader set_flag
    :initarg :set_flag
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass sync_start-response (<sync_start-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <sync_start-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'sync_start-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name move_robot-srv:<sync_start-response> is deprecated: use move_robot-srv:sync_start-response instead.")))

(cl:ensure-generic-function 'set_flag-val :lambda-list '(m))
(cl:defmethod set_flag-val ((m <sync_start-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader move_robot-srv:set_flag-val is deprecated.  Use move_robot-srv:set_flag instead.")
  (set_flag m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <sync_start-response>) ostream)
  "Serializes a message object of type '<sync_start-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'set_flag) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <sync_start-response>) istream)
  "Deserializes a message object of type '<sync_start-response>"
    (cl:setf (cl:slot-value msg 'set_flag) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<sync_start-response>)))
  "Returns string type for a service object of type '<sync_start-response>"
  "move_robot/sync_startResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'sync_start-response)))
  "Returns string type for a service object of type 'sync_start-response"
  "move_robot/sync_startResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<sync_start-response>)))
  "Returns md5sum for a message object of type '<sync_start-response>"
  "f28b7d6543382a4046cd196e0567aca4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'sync_start-response)))
  "Returns md5sum for a message object of type 'sync_start-response"
  "f28b7d6543382a4046cd196e0567aca4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<sync_start-response>)))
  "Returns full string definition for message of type '<sync_start-response>"
  (cl:format cl:nil "bool set_flag~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'sync_start-response)))
  "Returns full string definition for message of type 'sync_start-response"
  (cl:format cl:nil "bool set_flag~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <sync_start-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <sync_start-response>))
  "Converts a ROS message object to a list"
  (cl:list 'sync_start-response
    (cl:cons ':set_flag (set_flag msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'sync_start)))
  'sync_start-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'sync_start)))
  'sync_start-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'sync_start)))
  "Returns string type for a service object of type '<sync_start>"
  "move_robot/sync_start")