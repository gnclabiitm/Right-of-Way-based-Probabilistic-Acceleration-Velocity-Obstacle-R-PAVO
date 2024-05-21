;; Auto-generated. Do not edit!


(when (boundp 'move_robot::sync_start)
  (if (not (find-package "MOVE_ROBOT"))
    (make-package "MOVE_ROBOT"))
  (shadow 'sync_start (find-package "MOVE_ROBOT")))
(unless (find-package "MOVE_ROBOT::SYNC_START")
  (make-package "MOVE_ROBOT::SYNC_START"))
(unless (find-package "MOVE_ROBOT::SYNC_STARTREQUEST")
  (make-package "MOVE_ROBOT::SYNC_STARTREQUEST"))
(unless (find-package "MOVE_ROBOT::SYNC_STARTRESPONSE")
  (make-package "MOVE_ROBOT::SYNC_STARTRESPONSE"))

(in-package "ROS")





(defclass move_robot::sync_startRequest
  :super ros::object
  :slots (_check_flag ))

(defmethod move_robot::sync_startRequest
  (:init
   (&key
    ((:check_flag __check_flag) nil)
    )
   (send-super :init)
   (setq _check_flag __check_flag)
   self)
  (:check_flag
   (&optional (__check_flag :null))
   (if (not (eq __check_flag :null)) (setq _check_flag __check_flag)) _check_flag)
  (:serialization-length
   ()
   (+
    ;; bool _check_flag
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _check_flag
       (if _check_flag (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _check_flag
     (setq _check_flag (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass move_robot::sync_startResponse
  :super ros::object
  :slots (_set_flag ))

(defmethod move_robot::sync_startResponse
  (:init
   (&key
    ((:set_flag __set_flag) nil)
    )
   (send-super :init)
   (setq _set_flag __set_flag)
   self)
  (:set_flag
   (&optional (__set_flag :null))
   (if (not (eq __set_flag :null)) (setq _set_flag __set_flag)) _set_flag)
  (:serialization-length
   ()
   (+
    ;; bool _set_flag
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _set_flag
       (if _set_flag (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _set_flag
     (setq _set_flag (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass move_robot::sync_start
  :super ros::object
  :slots ())

(setf (get move_robot::sync_start :md5sum-) "f28b7d6543382a4046cd196e0567aca4")
(setf (get move_robot::sync_start :datatype-) "move_robot/sync_start")
(setf (get move_robot::sync_start :request) move_robot::sync_startRequest)
(setf (get move_robot::sync_start :response) move_robot::sync_startResponse)

(defmethod move_robot::sync_startRequest
  (:response () (instance move_robot::sync_startResponse :init)))

(setf (get move_robot::sync_startRequest :md5sum-) "f28b7d6543382a4046cd196e0567aca4")
(setf (get move_robot::sync_startRequest :datatype-) "move_robot/sync_startRequest")
(setf (get move_robot::sync_startRequest :definition-)
      "bool check_flag
---
bool set_flag

")

(setf (get move_robot::sync_startResponse :md5sum-) "f28b7d6543382a4046cd196e0567aca4")
(setf (get move_robot::sync_startResponse :datatype-) "move_robot/sync_startResponse")
(setf (get move_robot::sync_startResponse :definition-)
      "bool check_flag
---
bool set_flag

")



(provide :move_robot/sync_start "f28b7d6543382a4046cd196e0567aca4")


