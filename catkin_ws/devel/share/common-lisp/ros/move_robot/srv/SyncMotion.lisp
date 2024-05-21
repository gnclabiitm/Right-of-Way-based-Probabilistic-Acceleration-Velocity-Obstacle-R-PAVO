; Auto-generated. Do not edit!


(cl:in-package move_robot-srv)


;//! \htmlinclude SyncMotion-request.msg.html

(cl:defclass <SyncMotion-request> (roslisp-msg-protocol:ros-message)
  ((trigger
    :reader trigger
    :initarg :trigger
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SyncMotion-request (<SyncMotion-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SyncMotion-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SyncMotion-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name move_robot-srv:<SyncMotion-request> is deprecated: use move_robot-srv:SyncMotion-request instead.")))

(cl:ensure-generic-function 'trigger-val :lambda-list '(m))
(cl:defmethod trigger-val ((m <SyncMotion-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader move_robot-srv:trigger-val is deprecated.  Use move_robot-srv:trigger instead.")
  (trigger m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SyncMotion-request>) ostream)
  "Serializes a message object of type '<SyncMotion-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'trigger) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SyncMotion-request>) istream)
  "Deserializes a message object of type '<SyncMotion-request>"
    (cl:setf (cl:slot-value msg 'trigger) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SyncMotion-request>)))
  "Returns string type for a service object of type '<SyncMotion-request>"
  "move_robot/SyncMotionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SyncMotion-request)))
  "Returns string type for a service object of type 'SyncMotion-request"
  "move_robot/SyncMotionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SyncMotion-request>)))
  "Returns md5sum for a message object of type '<SyncMotion-request>"
  "e22398fd32450a580e4c379bfee2569b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SyncMotion-request)))
  "Returns md5sum for a message object of type 'SyncMotion-request"
  "e22398fd32450a580e4c379bfee2569b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SyncMotion-request>)))
  "Returns full string definition for message of type '<SyncMotion-request>"
  (cl:format cl:nil "bool trigger~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SyncMotion-request)))
  "Returns full string definition for message of type 'SyncMotion-request"
  (cl:format cl:nil "bool trigger~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SyncMotion-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SyncMotion-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SyncMotion-request
    (cl:cons ':trigger (trigger msg))
))
;//! \htmlinclude SyncMotion-response.msg.html

(cl:defclass <SyncMotion-response> (roslisp-msg-protocol:ros-message)
  ((sync_success
    :reader sync_success
    :initarg :sync_success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SyncMotion-response (<SyncMotion-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SyncMotion-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SyncMotion-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name move_robot-srv:<SyncMotion-response> is deprecated: use move_robot-srv:SyncMotion-response instead.")))

(cl:ensure-generic-function 'sync_success-val :lambda-list '(m))
(cl:defmethod sync_success-val ((m <SyncMotion-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader move_robot-srv:sync_success-val is deprecated.  Use move_robot-srv:sync_success instead.")
  (sync_success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SyncMotion-response>) ostream)
  "Serializes a message object of type '<SyncMotion-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'sync_success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SyncMotion-response>) istream)
  "Deserializes a message object of type '<SyncMotion-response>"
    (cl:setf (cl:slot-value msg 'sync_success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SyncMotion-response>)))
  "Returns string type for a service object of type '<SyncMotion-response>"
  "move_robot/SyncMotionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SyncMotion-response)))
  "Returns string type for a service object of type 'SyncMotion-response"
  "move_robot/SyncMotionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SyncMotion-response>)))
  "Returns md5sum for a message object of type '<SyncMotion-response>"
  "e22398fd32450a580e4c379bfee2569b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SyncMotion-response)))
  "Returns md5sum for a message object of type 'SyncMotion-response"
  "e22398fd32450a580e4c379bfee2569b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SyncMotion-response>)))
  "Returns full string definition for message of type '<SyncMotion-response>"
  (cl:format cl:nil "bool sync_success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SyncMotion-response)))
  "Returns full string definition for message of type 'SyncMotion-response"
  (cl:format cl:nil "bool sync_success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SyncMotion-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SyncMotion-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SyncMotion-response
    (cl:cons ':sync_success (sync_success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SyncMotion)))
  'SyncMotion-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SyncMotion)))
  'SyncMotion-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SyncMotion)))
  "Returns string type for a service object of type '<SyncMotion>"
  "move_robot/SyncMotion")