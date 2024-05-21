; Auto-generated. Do not edit!


(cl:in-package move_robot-msg)


;//! \htmlinclude mission_plot_data.msg.html

(cl:defclass <mission_plot_data> (roslisp-msg-protocol:ros-message)
  ((t_mission
    :reader t_mission
    :initarg :t_mission
    :type cl:float
    :initform 0.0)
   (d_min
    :reader d_min
    :initarg :d_min
    :type cl:float
    :initform 0.0)
   (t_comp
    :reader t_comp
    :initarg :t_comp
    :type cl:float
    :initform 0.0)
   (a_rms
    :reader a_rms
    :initarg :a_rms
    :type cl:float
    :initform 0.0))
)

(cl:defclass mission_plot_data (<mission_plot_data>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <mission_plot_data>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'mission_plot_data)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name move_robot-msg:<mission_plot_data> is deprecated: use move_robot-msg:mission_plot_data instead.")))

(cl:ensure-generic-function 't_mission-val :lambda-list '(m))
(cl:defmethod t_mission-val ((m <mission_plot_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader move_robot-msg:t_mission-val is deprecated.  Use move_robot-msg:t_mission instead.")
  (t_mission m))

(cl:ensure-generic-function 'd_min-val :lambda-list '(m))
(cl:defmethod d_min-val ((m <mission_plot_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader move_robot-msg:d_min-val is deprecated.  Use move_robot-msg:d_min instead.")
  (d_min m))

(cl:ensure-generic-function 't_comp-val :lambda-list '(m))
(cl:defmethod t_comp-val ((m <mission_plot_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader move_robot-msg:t_comp-val is deprecated.  Use move_robot-msg:t_comp instead.")
  (t_comp m))

(cl:ensure-generic-function 'a_rms-val :lambda-list '(m))
(cl:defmethod a_rms-val ((m <mission_plot_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader move_robot-msg:a_rms-val is deprecated.  Use move_robot-msg:a_rms instead.")
  (a_rms m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <mission_plot_data>) ostream)
  "Serializes a message object of type '<mission_plot_data>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 't_mission))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'd_min))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 't_comp))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'a_rms))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <mission_plot_data>) istream)
  "Deserializes a message object of type '<mission_plot_data>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 't_mission) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'd_min) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 't_comp) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'a_rms) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<mission_plot_data>)))
  "Returns string type for a message object of type '<mission_plot_data>"
  "move_robot/mission_plot_data")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'mission_plot_data)))
  "Returns string type for a message object of type 'mission_plot_data"
  "move_robot/mission_plot_data")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<mission_plot_data>)))
  "Returns md5sum for a message object of type '<mission_plot_data>"
  "cc7d6c5759137cf046d2c010b9c936e4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'mission_plot_data)))
  "Returns md5sum for a message object of type 'mission_plot_data"
  "cc7d6c5759137cf046d2c010b9c936e4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<mission_plot_data>)))
  "Returns full string definition for message of type '<mission_plot_data>"
  (cl:format cl:nil "float64 t_mission~%float64 d_min~%float64 t_comp~%float64 a_rms~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'mission_plot_data)))
  "Returns full string definition for message of type 'mission_plot_data"
  (cl:format cl:nil "float64 t_mission~%float64 d_min~%float64 t_comp~%float64 a_rms~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <mission_plot_data>))
  (cl:+ 0
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <mission_plot_data>))
  "Converts a ROS message object to a list"
  (cl:list 'mission_plot_data
    (cl:cons ':t_mission (t_mission msg))
    (cl:cons ':d_min (d_min msg))
    (cl:cons ':t_comp (t_comp msg))
    (cl:cons ':a_rms (a_rms msg))
))
