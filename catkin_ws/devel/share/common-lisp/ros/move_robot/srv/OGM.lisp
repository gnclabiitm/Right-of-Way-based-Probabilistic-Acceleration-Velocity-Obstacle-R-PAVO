; Auto-generated. Do not edit!


(cl:in-package move_robot-srv)


;//! \htmlinclude OGM-request.msg.html

(cl:defclass <OGM-request> (roslisp-msg-protocol:ros-message)
  ((tb3_id
    :reader tb3_id
    :initarg :tb3_id
    :type cl:string
    :initform "")
   (ogm_node_num
    :reader ogm_node_num
    :initarg :ogm_node_num
    :type cl:string
    :initform ""))
)

(cl:defclass OGM-request (<OGM-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OGM-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OGM-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name move_robot-srv:<OGM-request> is deprecated: use move_robot-srv:OGM-request instead.")))

(cl:ensure-generic-function 'tb3_id-val :lambda-list '(m))
(cl:defmethod tb3_id-val ((m <OGM-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader move_robot-srv:tb3_id-val is deprecated.  Use move_robot-srv:tb3_id instead.")
  (tb3_id m))

(cl:ensure-generic-function 'ogm_node_num-val :lambda-list '(m))
(cl:defmethod ogm_node_num-val ((m <OGM-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader move_robot-srv:ogm_node_num-val is deprecated.  Use move_robot-srv:ogm_node_num instead.")
  (ogm_node_num m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OGM-request>) ostream)
  "Serializes a message object of type '<OGM-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'tb3_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'tb3_id))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'ogm_node_num))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'ogm_node_num))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OGM-request>) istream)
  "Deserializes a message object of type '<OGM-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'tb3_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'tb3_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ogm_node_num) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'ogm_node_num) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OGM-request>)))
  "Returns string type for a service object of type '<OGM-request>"
  "move_robot/OGMRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OGM-request)))
  "Returns string type for a service object of type 'OGM-request"
  "move_robot/OGMRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OGM-request>)))
  "Returns md5sum for a message object of type '<OGM-request>"
  "a3ef7e635198434f8c5d1fcc111d7ba4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OGM-request)))
  "Returns md5sum for a message object of type 'OGM-request"
  "a3ef7e635198434f8c5d1fcc111d7ba4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OGM-request>)))
  "Returns full string definition for message of type '<OGM-request>"
  (cl:format cl:nil "string tb3_id~%string ogm_node_num~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OGM-request)))
  "Returns full string definition for message of type 'OGM-request"
  (cl:format cl:nil "string tb3_id~%string ogm_node_num~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OGM-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'tb3_id))
     4 (cl:length (cl:slot-value msg 'ogm_node_num))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OGM-request>))
  "Converts a ROS message object to a list"
  (cl:list 'OGM-request
    (cl:cons ':tb3_id (tb3_id msg))
    (cl:cons ':ogm_node_num (ogm_node_num msg))
))
;//! \htmlinclude OGM-response.msg.html

(cl:defclass <OGM-response> (roslisp-msg-protocol:ros-message)
  ((dummy
    :reader dummy
    :initarg :dummy
    :type cl:string
    :initform ""))
)

(cl:defclass OGM-response (<OGM-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OGM-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OGM-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name move_robot-srv:<OGM-response> is deprecated: use move_robot-srv:OGM-response instead.")))

(cl:ensure-generic-function 'dummy-val :lambda-list '(m))
(cl:defmethod dummy-val ((m <OGM-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader move_robot-srv:dummy-val is deprecated.  Use move_robot-srv:dummy instead.")
  (dummy m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OGM-response>) ostream)
  "Serializes a message object of type '<OGM-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'dummy))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'dummy))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OGM-response>) istream)
  "Deserializes a message object of type '<OGM-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'dummy) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'dummy) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OGM-response>)))
  "Returns string type for a service object of type '<OGM-response>"
  "move_robot/OGMResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OGM-response)))
  "Returns string type for a service object of type 'OGM-response"
  "move_robot/OGMResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OGM-response>)))
  "Returns md5sum for a message object of type '<OGM-response>"
  "a3ef7e635198434f8c5d1fcc111d7ba4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OGM-response)))
  "Returns md5sum for a message object of type 'OGM-response"
  "a3ef7e635198434f8c5d1fcc111d7ba4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OGM-response>)))
  "Returns full string definition for message of type '<OGM-response>"
  (cl:format cl:nil "string dummy~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OGM-response)))
  "Returns full string definition for message of type 'OGM-response"
  (cl:format cl:nil "string dummy~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OGM-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'dummy))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OGM-response>))
  "Converts a ROS message object to a list"
  (cl:list 'OGM-response
    (cl:cons ':dummy (dummy msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'OGM)))
  'OGM-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'OGM)))
  'OGM-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OGM)))
  "Returns string type for a service object of type '<OGM>"
  "move_robot/OGM")