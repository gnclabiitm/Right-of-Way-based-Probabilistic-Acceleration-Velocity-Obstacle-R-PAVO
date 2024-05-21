;; Auto-generated. Do not edit!


(when (boundp 'move_robot::xycoord)
  (if (not (find-package "MOVE_ROBOT"))
    (make-package "MOVE_ROBOT"))
  (shadow 'xycoord (find-package "MOVE_ROBOT")))
(unless (find-package "MOVE_ROBOT::XYCOORD")
  (make-package "MOVE_ROBOT::XYCOORD"))
(unless (find-package "MOVE_ROBOT::XYCOORDREQUEST")
  (make-package "MOVE_ROBOT::XYCOORDREQUEST"))
(unless (find-package "MOVE_ROBOT::XYCOORDRESPONSE")
  (make-package "MOVE_ROBOT::XYCOORDRESPONSE"))

(in-package "ROS")





(defclass move_robot::xycoordRequest
  :super ros::object
  :slots (_occ_theta _l _w ))

(defmethod move_robot::xycoordRequest
  (:init
   (&key
    ((:occ_theta __occ_theta) 0.0)
    ((:l __l) 0.0)
    ((:w __w) 0.0)
    )
   (send-super :init)
   (setq _occ_theta (float __occ_theta))
   (setq _l (float __l))
   (setq _w (float __w))
   self)
  (:occ_theta
   (&optional __occ_theta)
   (if __occ_theta (setq _occ_theta __occ_theta)) _occ_theta)
  (:l
   (&optional __l)
   (if __l (setq _l __l)) _l)
  (:w
   (&optional __w)
   (if __w (setq _w __w)) _w)
  (:serialization-length
   ()
   (+
    ;; float64 _occ_theta
    8
    ;; float64 _l
    8
    ;; float64 _w
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _occ_theta
       (sys::poke _occ_theta (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _l
       (sys::poke _l (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _w
       (sys::poke _w (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _occ_theta
     (setq _occ_theta (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _l
     (setq _l (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _w
     (setq _w (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass move_robot::xycoordResponse
  :super ros::object
  :slots (_x2 _y2 ))

(defmethod move_robot::xycoordResponse
  (:init
   (&key
    ((:x2 __x2) 0.0)
    ((:y2 __y2) 0.0)
    )
   (send-super :init)
   (setq _x2 (float __x2))
   (setq _y2 (float __y2))
   self)
  (:x2
   (&optional __x2)
   (if __x2 (setq _x2 __x2)) _x2)
  (:y2
   (&optional __y2)
   (if __y2 (setq _y2 __y2)) _y2)
  (:serialization-length
   ()
   (+
    ;; float64 _x2
    8
    ;; float64 _y2
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _x2
       (sys::poke _x2 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y2
       (sys::poke _y2 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _x2
     (setq _x2 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y2
     (setq _y2 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass move_robot::xycoord
  :super ros::object
  :slots ())

(setf (get move_robot::xycoord :md5sum-) "468f9ac078489289ae3d0296932d3837")
(setf (get move_robot::xycoord :datatype-) "move_robot/xycoord")
(setf (get move_robot::xycoord :request) move_robot::xycoordRequest)
(setf (get move_robot::xycoord :response) move_robot::xycoordResponse)

(defmethod move_robot::xycoordRequest
  (:response () (instance move_robot::xycoordResponse :init)))

(setf (get move_robot::xycoordRequest :md5sum-) "468f9ac078489289ae3d0296932d3837")
(setf (get move_robot::xycoordRequest :datatype-) "move_robot/xycoordRequest")
(setf (get move_robot::xycoordRequest :definition-)
      "float64 occ_theta
float64 l
float64 w 
---
float64 x2
float64 y2
")

(setf (get move_robot::xycoordResponse :md5sum-) "468f9ac078489289ae3d0296932d3837")
(setf (get move_robot::xycoordResponse :datatype-) "move_robot/xycoordResponse")
(setf (get move_robot::xycoordResponse :definition-)
      "float64 occ_theta
float64 l
float64 w 
---
float64 x2
float64 y2
")



(provide :move_robot/xycoord "468f9ac078489289ae3d0296932d3837")


