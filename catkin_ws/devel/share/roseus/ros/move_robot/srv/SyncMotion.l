;; Auto-generated. Do not edit!


(when (boundp 'move_robot::SyncMotion)
  (if (not (find-package "MOVE_ROBOT"))
    (make-package "MOVE_ROBOT"))
  (shadow 'SyncMotion (find-package "MOVE_ROBOT")))
(unless (find-package "MOVE_ROBOT::SYNCMOTION")
  (make-package "MOVE_ROBOT::SYNCMOTION"))
(unless (find-package "MOVE_ROBOT::SYNCMOTIONREQUEST")
  (make-package "MOVE_ROBOT::SYNCMOTIONREQUEST"))
(unless (find-package "MOVE_ROBOT::SYNCMOTIONRESPONSE")
  (make-package "MOVE_ROBOT::SYNCMOTIONRESPONSE"))

(in-package "ROS")





(defclass move_robot::SyncMotionRequest
  :super ros::object
  :slots (_trigger ))

(defmethod move_robot::SyncMotionRequest
  (:init
   (&key
    ((:trigger __trigger) nil)
    )
   (send-super :init)
   (setq _trigger __trigger)
   self)
  (:trigger
   (&optional (__trigger :null))
   (if (not (eq __trigger :null)) (setq _trigger __trigger)) _trigger)
  (:serialization-length
   ()
   (+
    ;; bool _trigger
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _trigger
       (if _trigger (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _trigger
     (setq _trigger (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass move_robot::SyncMotionResponse
  :super ros::object
  :slots (_sync_success ))

(defmethod move_robot::SyncMotionResponse
  (:init
   (&key
    ((:sync_success __sync_success) nil)
    )
   (send-super :init)
   (setq _sync_success __sync_success)
   self)
  (:sync_success
   (&optional (__sync_success :null))
   (if (not (eq __sync_success :null)) (setq _sync_success __sync_success)) _sync_success)
  (:serialization-length
   ()
   (+
    ;; bool _sync_success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _sync_success
       (if _sync_success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _sync_success
     (setq _sync_success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass move_robot::SyncMotion
  :super ros::object
  :slots ())

(setf (get move_robot::SyncMotion :md5sum-) "e22398fd32450a580e4c379bfee2569b")
(setf (get move_robot::SyncMotion :datatype-) "move_robot/SyncMotion")
(setf (get move_robot::SyncMotion :request) move_robot::SyncMotionRequest)
(setf (get move_robot::SyncMotion :response) move_robot::SyncMotionResponse)

(defmethod move_robot::SyncMotionRequest
  (:response () (instance move_robot::SyncMotionResponse :init)))

(setf (get move_robot::SyncMotionRequest :md5sum-) "e22398fd32450a580e4c379bfee2569b")
(setf (get move_robot::SyncMotionRequest :datatype-) "move_robot/SyncMotionRequest")
(setf (get move_robot::SyncMotionRequest :definition-)
      "bool trigger
---
bool sync_success

")

(setf (get move_robot::SyncMotionResponse :md5sum-) "e22398fd32450a580e4c379bfee2569b")
(setf (get move_robot::SyncMotionResponse :datatype-) "move_robot/SyncMotionResponse")
(setf (get move_robot::SyncMotionResponse :definition-)
      "bool trigger
---
bool sync_success

")



(provide :move_robot/SyncMotion "e22398fd32450a580e4c379bfee2569b")


